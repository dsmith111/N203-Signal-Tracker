"""Generate N203 component models for enclosure layout.

The resulting STEP assemblies are intended for mechanical clearance and visual
layout in CAD. They are deliberately simple and are not PCB footprints.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cadquery as cq


# Shared construction and first-print clearance values, in millimetres.
PCB_THICKNESS = 1.60
XY_CLEARANCE_PER_SIDE = 0.50
Z_CLEARANCE_BELOW = 0.50
Z_CLEARANCE_ABOVE = 0.80

# MSP3520 / ILI9488 drawing dimensions.
DISPLAY_PCB_LENGTH = 98.00
DISPLAY_PCB_WIDTH = 56.34
DISPLAY_PCB_THICKNESS = 1.60
DISPLAY_HOLE_SPACING_X = 92.00
DISPLAY_HOLE_SPACING_Y = 49.50
DISPLAY_HOLE_DIAMETER = 3.20
DISPLAY_LCD_LENGTH = 84.96
DISPLAY_LCD_WIDTH = 55.50
DISPLAY_ACTIVE_LENGTH = 73.44
DISPLAY_ACTIVE_WIDTH = 48.96
DISPLAY_FRONT_TOTAL_Z = 5.80
DISPLAY_HEADER_BOTTOM_Z = -8.38
# LCDWiki MSP3520_Size.pdf: the 14-pin header row is 2.00 mm from the short PCB
# edge and centred across the 56.34 mm width (11.66 mm to the first pin).
DISPLAY_HEADER_EDGE_OFFSET = 2.00
# The LCD frame is centred on the PCB (6.54 / 6.50 mm end margins) but the
# active area sits 9.95 mm from the non-header end and 14.61 mm from the
# header end, i.e. 2.33 mm toward the non-header (+X) end.
DISPLAY_ACTIVE_OFFSET_X = 2.33
# Rear microSD socket envelope. Position scaled from the drawing's back view;
# the 3.00 mm figure is the drawing's "3.00 Max (SMD)" rear keep-out.
DISPLAY_SD_LENGTH = 26.00
DISPLAY_SD_WIDTH = 17.00
DISPLAY_SD_HEIGHT = 2.20
DISPLAY_SD_CENTER_X = 29.50
DISPLAY_SD_CENTER_Y = 17.00

# YD/VCC-GND Studio dual-USB ESP32-S3 clone. Only the PCB envelope is based on
# the matching board description; component locations remain approximate.
S3_PCB_LENGTH = 63.50
S3_PCB_WIDTH = 27.94

# Generic 16-pin ESP32-C3 SuperMini.
C3_PCB_LENGTH = 22.00
C3_PCB_WIDTH = 18.00
C3_USB_OVERHANG = 2.00

# Ai-Thinker RD-03D V1. The datasheet specifies X/Y but not assembled height.
RD03D_PCB_LENGTH = 44.00
RD03D_PCB_WIDTH = 15.00
RD03D_PCB_THICKNESS_APPROX = 1.20
# Rear 1x4 1.25 mm side-entry connector. Footprint scaled from the size chart
# (solder tabs reach 9.40 mm in from the board end, body about 8.85 mm wide).
RD03D_CONNECTOR_LENGTH = 8.00
RD03D_CONNECTOR_WIDTH = 8.85
RD03D_CONNECTOR_HEIGHT_APPROX = 4.00
RD03D_REAR_COMPONENT_HEIGHT_APPROX = 1.60

# Additional enclosure components. Dimensions marked approximate should be
# replaced with measurements from the exact physical part before finalizing a
# printed enclosure.
GY521_PCB_LENGTH_APPROX = 21.00
GY521_PCB_WIDTH_APPROX = 16.00
GY521_PCB_THICKNESS_APPROX = 1.60

JLJLUP_LIPO_LENGTH = 52.00
JLJLUP_LIPO_WIDTH = 34.00
JLJLUP_LIPO_THICKNESS = 10.00

C4001_PCB_LENGTH = 26.00
C4001_PCB_WIDTH = 30.00
C4001_PCB_THICKNESS_APPROX = 1.60
C4001_MOUNTING_HOLE_DIAMETER = 2.00
C4001_MOUNTING_HOLE_SPACING = 22.00

BQ24074_PCB_LENGTH = 38.10
BQ24074_PCB_WIDTH = 33.02
BQ24074_PCB_THICKNESS_APPROX = 1.60
BQ24074_CORNER_RADIUS = 2.54
BQ24074_MOUNTING_HOLE_DIAMETER = 2.50
BQ24074_MOUNTING_SPACING_X = 33.02
BQ24074_MOUNTING_SPACING_Y = 27.94

# Pololu S13V20F5 (item 4085) envelope, from the manufacturer's published model.
POLOLU_S13V20F5_LENGTH = 8.89
POLOLU_S13V20F5_WIDTH = 12.07
POLOLU_S13V20F5_BOTTOM_Z = -1.35
POLOLU_S13V20F5_TOP_Z = 4.02
POLOLU_S13V20F5_PCB_NOMINAL = 1.00

# Optional local copy of the manufacturer's STEP; not kept in the repository.
POLOLU_S13V20F5_VENDOR_STEP = (
    Path(__file__).resolve().parent
    / "vendor"
    / "pololu"
    / "s13v20f5_item4085.step"
)


@dataclass(frozen=True)
class Model:
    slug: str
    description: str
    assembly: cq.Assembly
    source_accuracy: str = "approximate enclosure geometry"
    keepout_xy_per_side: float = XY_CLEARANCE_PER_SIDE
    keepout_z_below: float = Z_CLEARANCE_BELOW
    keepout_z_above: float = Z_CLEARANCE_ABOVE

    @property
    def compound(self) -> cq.Shape:
        return self.assembly.toCompound()


def box(
    length: float,
    width: float,
    height: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    bottom_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((center_x, center_y, bottom_z))
    )


def cylinder(
    diameter: float,
    height: float,
    *,
    center_x: float,
    center_y: float,
    bottom_z: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(diameter / 2.0)
        .extrude(height)
        .translate((center_x, center_y, bottom_z))
    )


def pcb_with_holes(
    length: float,
    width: float,
    thickness: float,
    points: list[tuple[float, float]],
    hole_diameter: float,
) -> cq.Workplane:
    pcb = box(length, width, thickness)
    if points:
        pcb = (
            pcb.faces(">Z")
            .workplane()
            .pushPoints(points)
            .hole(hole_diameter, thickness)
        )
    return pcb


def rounded_pcb_with_holes(
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
    points: list[tuple[float, float]],
    hole_diameter: float,
) -> cq.Workplane:
    pcb = box(length, width, thickness).edges("|Z").fillet(corner_radius)
    if points:
        pcb = (
            pcb.faces(">Z")
            .workplane()
            .pushPoints(points)
            .hole(hole_diameter, thickness)
        )
    return pcb


def build_display() -> Model:
    holes = [
        (x, y)
        for x in (-DISPLAY_HOLE_SPACING_X / 2.0, DISPLAY_HOLE_SPACING_X / 2.0)
        for y in (-DISPLAY_HOLE_SPACING_Y / 2.0, DISPLAY_HOLE_SPACING_Y / 2.0)
    ]
    pcb = pcb_with_holes(
        DISPLAY_PCB_LENGTH,
        DISPLAY_PCB_WIDTH,
        DISPLAY_PCB_THICKNESS,
        holes,
        DISPLAY_HOLE_DIAMETER,
    )
    lcd_stack = box(
        DISPLAY_LCD_LENGTH,
        DISPLAY_LCD_WIDTH,
        DISPLAY_FRONT_TOTAL_Z - DISPLAY_PCB_THICKNESS,
        bottom_z=DISPLAY_PCB_THICKNESS,
    )
    active_surface = box(
        DISPLAY_ACTIVE_LENGTH,
        DISPLAY_ACTIVE_WIDTH,
        0.08,
        center_x=DISPLAY_ACTIVE_OFFSET_X,
        bottom_z=DISPLAY_FRONT_TOTAL_Z - 0.08,
    )
    sd_socket = box(
        DISPLAY_SD_LENGTH,
        DISPLAY_SD_WIDTH,
        DISPLAY_SD_HEIGHT,
        center_x=DISPLAY_SD_CENTER_X,
        center_y=DISPLAY_SD_CENTER_Y,
        bottom_z=-DISPLAY_SD_HEIGHT,
    )

    # The 14-pin connector is simplified to a shrouded row and pins. Row
    # position follows the LCDWiki drawing; pin 1 (VCC) is at +Y, T_IRQ at -Y.
    header_x = -(DISPLAY_PCB_LENGTH / 2.0) + DISPLAY_HEADER_EDGE_OFFSET
    header_body = box(
        2.54,
        35.56,
        2.54,
        center_x=header_x,
        bottom_z=-2.54,
    )
    header_pins = [
        cylinder(
            0.64,
            abs(DISPLAY_HEADER_BOTTOM_Z),
            center_x=header_x,
            center_y=(index - 6.5) * 2.54,
            bottom_z=DISPLAY_HEADER_BOTTOM_Z,
        )
        for index in range(14)
    ]

    assy = cq.Assembly(name="n203_msp3520_display_approx")
    assy.add(pcb, name="pcb_98x56_34", color=cq.Color(0.05, 0.25, 0.12))
    assy.add(lcd_stack, name="lcd_touch_stack", color=cq.Color(0.12, 0.12, 0.12))
    assy.add(active_surface, name="active_area_73_44x48_96", color=cq.Color(0.02, 0.03, 0.03))
    assy.add(sd_socket, name="rear_microsd_socket_approx", color=cq.Color(0.62, 0.64, 0.66))
    assy.add(header_body, name="header_body_approx", color=cq.Color(0.08, 0.08, 0.08))
    for index, pin in enumerate(header_pins, start=1):
        assy.add(pin, name=f"header_pin_{index:02d}", color=cq.Color(0.75, 0.58, 0.16))
    return Model(
        "n203_msp3520_display_approx",
        "MSP3520 display with PCB, hole pattern, header row, and Z envelope from the LCDWiki drawing",
        assy,
    )


def build_s3() -> Model:
    pin_x = [(index - 10.5) * 2.54 for index in range(22)]
    pin_y = (-12.70, 12.70)
    holes = [(x, y) for x in pin_x for y in pin_y]
    pcb = pcb_with_holes(S3_PCB_LENGTH, S3_PCB_WIDTH, PCB_THICKNESS, holes, 1.00)

    module = box(25.50, 18.00, 3.20, center_x=13.0, bottom_z=PCB_THICKNESS)
    component_zone = box(22.0, 22.0, 2.30, center_x=-10.0, bottom_z=PCB_THICKNESS)

    # Two Type-C bodies protrude approximately 1.2 mm beyond the USB end.
    usb_body_length = 6.0
    usb_center_x = -(S3_PCB_LENGTH / 2.0) + (usb_body_length / 2.0) - 1.2
    usb_left = box(usb_body_length, 7.40, 3.20, center_x=usb_center_x, center_y=-5.0, bottom_z=PCB_THICKNESS)
    usb_right = box(usb_body_length, 7.40, 3.20, center_x=usb_center_x, center_y=5.0, bottom_z=PCB_THICKNESS)
    boot = box(4.50, 3.50, 2.50, center_x=-22.0, center_y=-9.5, bottom_z=PCB_THICKNESS)
    reset = box(4.50, 3.50, 2.50, center_x=-22.0, center_y=9.5, bottom_z=PCB_THICKNESS)
    ufl = cylinder(2.50, 1.50, center_x=23.0, center_y=8.0, bottom_z=PCB_THICKNESS)

    assy = cq.Assembly(name="n203_yd_esp32_s3_approx")
    assy.add(pcb, name="pcb_63_5x27_94", color=cq.Color(0.08, 0.18, 0.32))
    assy.add(module, name="esp32_s3_wroom_envelope", color=cq.Color(0.55, 0.58, 0.60))
    assy.add(component_zone, name="support_components_envelope", color=cq.Color(0.10, 0.10, 0.10))
    assy.add(usb_left, name="usb_c_com_approx", color=cq.Color(0.65, 0.67, 0.69))
    assy.add(usb_right, name="usb_c_usb_approx", color=cq.Color(0.65, 0.67, 0.69))
    assy.add(boot, name="boot_button_approx", color=cq.Color(0.18, 0.18, 0.18))
    assy.add(reset, name="reset_button_approx", color=cq.Color(0.18, 0.18, 0.18))
    assy.add(ufl, name="ufl_connector_approx", color=cq.Color(0.78, 0.68, 0.28))
    return Model(
        "n203_yd_esp32_s3_approx",
        "YD-style 44-pin dual-USB ESP32-S3 clone; only PCB envelope is source-based",
        assy,
    )


def build_c3() -> Model:
    pin_x = [(index - 3.5) * 2.54 for index in range(8)]
    pin_y = (-7.62, 7.62)
    holes = [(x, y) for x in pin_x for y in pin_y]
    pcb = pcb_with_holes(C3_PCB_LENGTH, C3_PCB_WIDTH, PCB_THICKNESS, holes, 1.00)

    usb_body_length = 6.0
    usb_center_x = -(C3_PCB_LENGTH / 2.0) + (usb_body_length / 2.0) - C3_USB_OVERHANG
    usb = box(usb_body_length, 7.40, 3.20, center_x=usb_center_x, bottom_z=PCB_THICKNESS)
    soc = box(5.20, 5.20, 1.00, center_x=0.5, center_y=0.5, bottom_z=PCB_THICKNESS)
    antenna = box(4.80, 2.20, 2.20, center_x=8.0, bottom_z=PCB_THICKNESS)
    boot = box(3.20, 2.60, 1.80, center_x=-5.7, center_y=-5.4, bottom_z=PCB_THICKNESS)
    reset = box(3.20, 2.60, 1.80, center_x=-5.7, center_y=5.4, bottom_z=PCB_THICKNESS)
    regulator_zone = box(5.0, 4.0, 1.40, center_x=3.8, center_y=-4.8, bottom_z=PCB_THICKNESS)

    assy = cq.Assembly(name="n203_esp32_c3_supermini_approx")
    assy.add(pcb, name="pcb_22x18", color=cq.Color(0.08, 0.18, 0.32))
    assy.add(usb, name="usb_c_approx", color=cq.Color(0.65, 0.67, 0.69))
    assy.add(soc, name="esp32_c3_envelope", color=cq.Color(0.12, 0.12, 0.12))
    assy.add(antenna, name="ceramic_antenna_approx", color=cq.Color(0.82, 0.78, 0.66))
    assy.add(boot, name="boot_button_approx", color=cq.Color(0.18, 0.18, 0.18))
    assy.add(reset, name="reset_button_approx", color=cq.Color(0.18, 0.18, 0.18))
    assy.add(regulator_zone, name="support_components_envelope", color=cq.Color(0.12, 0.12, 0.12))
    return Model(
        "n203_esp32_c3_supermini_approx",
        "Generic 16-pin ESP32-C3 SuperMini with approximate component placement",
        assy,
    )


def build_rd03d() -> Model:
    # Ai-Thinker Rd-03D specification V1.0.0, figures 3 and 4: the antenna
    # patches and radar SoC are on the front (+Z here); the 1x4 1.25 mm
    # connector and the support components are on the back (-Z here). The
    # connector sits behind the 2x2 patch group at the -X end and mates from
    # that end of the board.
    pcb = box(RD03D_PCB_LENGTH, RD03D_PCB_WIDTH, RD03D_PCB_THICKNESS_APPROX)
    front_z = RD03D_PCB_THICKNESS_APPROX

    connector = box(
        RD03D_CONNECTOR_LENGTH,
        RD03D_CONNECTOR_WIDTH,
        RD03D_CONNECTOR_HEIGHT_APPROX,
        center_x=-(RD03D_PCB_LENGTH / 2.0) + RD03D_CONNECTOR_LENGTH / 2.0,
        bottom_z=-RD03D_CONNECTOR_HEIGHT_APPROX,
    )
    rear_components = box(
        30.0,
        12.0,
        RD03D_REAR_COMPONENT_HEIGHT_APPROX,
        center_x=4.0,
        bottom_z=-RD03D_REAR_COMPONENT_HEIGHT_APPROX,
    )
    radar_soc = box(5.20, 5.20, 1.00, center_x=-3.0, center_y=2.5, bottom_z=front_z)

    # Schematic antenna patches: a 2x2 group at the connector (-X) end and a
    # two-element group at the +X end, as drawn in the specification.
    patch_points = [
        (-17.5, 3.4),
        (-17.5, -3.4),
        (-10.5, 3.4),
        (-10.5, -3.4),
        (7.0, -2.0),
        (14.0, -2.0),
    ]
    patches = [
        box(4.0, 3.6, 0.08, center_x=x, center_y=y, bottom_z=front_z)
        for x, y in patch_points
    ]

    assy = cq.Assembly(name="n203_rd03d_v1_approx")
    assy.add(pcb, name="pcb_44x15", color=cq.Color(0.05, 0.05, 0.05))
    assy.add(connector, name="rear_connector_1x4_1_25_approx", color=cq.Color(0.88, 0.88, 0.84))
    assy.add(rear_components, name="rear_components_envelope", color=cq.Color(0.16, 0.16, 0.16))
    assy.add(radar_soc, name="front_radar_soc_envelope", color=cq.Color(0.12, 0.12, 0.12))
    for index, patch in enumerate(patches, start=1):
        assy.add(patch, name=f"front_antenna_patch_{index}_schematic", color=cq.Color(0.80, 0.62, 0.20))
    return Model(
        "n203_rd03d_v1_approx",
        "RD-03D V1: verified 44 x 15 mm outline, front antenna face, rear connector; heights schematic",
        assy,
    )


def build_pololu_s13v20f5() -> Model:
    """Envelope model of the Pololu S13V20F5 (item 4085).

    Pololu publishes a STEP file for this board, but it is their model and is
    not redistributed here.  This stand-in has the same outline and the same
    extent above and below the PCB (8.89 x 12.07 mm, Z from -1.35 to +4.02 mm),
    which is all the enclosure design relies on.  The split between PCB and
    component blocks is nominal.
    """
    pcb = box(POLOLU_S13V20F5_LENGTH, POLOLU_S13V20F5_WIDTH, POLOLU_S13V20F5_PCB_NOMINAL)
    top_parts = box(
        POLOLU_S13V20F5_LENGTH - 1.60,
        POLOLU_S13V20F5_WIDTH - 1.60,
        POLOLU_S13V20F5_TOP_Z - POLOLU_S13V20F5_PCB_NOMINAL,
        bottom_z=POLOLU_S13V20F5_PCB_NOMINAL,
    )
    back_parts = box(
        POLOLU_S13V20F5_LENGTH - 2.40,
        POLOLU_S13V20F5_WIDTH - 5.00,
        -POLOLU_S13V20F5_BOTTOM_Z,
        bottom_z=POLOLU_S13V20F5_BOTTOM_Z,
    )

    assy = cq.Assembly(name="n203_pololu_s13v20f5_item4085_approx")
    assy.add(pcb, name="pcb_outline", color=cq.Color(0.15, 0.35, 0.18))
    assy.add(top_parts, name="front_components_envelope", color=cq.Color(0.12, 0.12, 0.12))
    assy.add(back_parts, name="back_components_envelope", color=cq.Color(0.12, 0.12, 0.12))
    return Model(
        "n203_pololu_s13v20f5_item4085_approx",
        "Pololu item 4085 S13V20F5, envelope with the published outline and height",
        assy,
        source_accuracy="outline and overall height match the manufacturer's model; internal split approximate",
    )


def build_pololu_s13v20f5_official() -> Model | None:
    """Optional: import the manufacturer's STEP if it has been downloaded.

    The file is not kept in the repository; see vendor/pololu/README.md.  The
    bracer designs use the envelope model above, so this is only a nicer
    visual reference for local CAD work.
    """
    if not POLOLU_S13V20F5_VENDOR_STEP.is_file():
        return None

    # Pololu's model uses one PCB corner as X/Y zero and already places the PCB
    # around Z=0. Center X/Y to match the rest of this package without changing
    # the manufacturer's geometry or its useful Z datum.
    imported = cq.importers.importStep(str(POLOLU_S13V20F5_VENDOR_STEP))
    bb = imported.val().BoundingBox()
    centered = imported.translate(
        (-(bb.xmin + bb.xmax) / 2.0, -(bb.ymin + bb.ymax) / 2.0, 0.0)
    )

    assy = cq.Assembly(name="n203_pololu_s13v20f5_item4085_official")
    assy.add(centered, name="official_pololu_s13v20f5", color=cq.Color(0.15, 0.35, 0.18))
    return Model(
        "n203_pololu_s13v20f5_item4085_official",
        "Pololu item 4085 S13V20F5, imported from the official manufacturer STEP",
        assy,
        source_accuracy="official manufacturer STEP; X/Y recentered only",
    )


def build_gy521_mpu6050() -> Model:
    mounting_holes = [(-7.5, 5.35), (7.5, 5.35)]
    pcb = pcb_with_holes(
        GY521_PCB_LENGTH_APPROX,
        GY521_PCB_WIDTH_APPROX,
        GY521_PCB_THICKNESS_APPROX,
        mounting_holes,
        3.00,
    )
    header_points = [((index - 3.5) * 2.54, -6.35) for index in range(8)]
    pcb = (
        pcb.faces(">Z")
        .workplane()
        .pushPoints(header_points)
        .hole(1.00, GY521_PCB_THICKNESS_APPROX)
    )
    imu = box(4.00, 4.00, 0.90, center_y=0.5, bottom_z=GY521_PCB_THICKNESS_APPROX)
    support_zone = box(
        9.0,
        5.0,
        2.20,
        center_y=-2.6,
        bottom_z=GY521_PCB_THICKNESS_APPROX,
    )

    assy = cq.Assembly(name="n203_gy521_mpu6050_approx")
    assy.add(pcb, name="gy521_pcb_21x16_approx", color=cq.Color(0.10, 0.25, 0.50))
    assy.add(imu, name="mpu6050_package_approx", color=cq.Color(0.10, 0.10, 0.10))
    assy.add(support_zone, name="support_components_envelope", color=cq.Color(0.16, 0.16, 0.16))
    return Model(
        "n203_gy521_mpu6050_approx",
        "Common GY-521 MPU-6050 board envelope; header pins are intentionally omitted",
        assy,
        source_accuracy="generic GY-521 approximation; verify the supplied Fasvicna board",
    )


def build_jljlup_lipo() -> Model:
    pouch = (
        box(JLJLUP_LIPO_LENGTH, JLJLUP_LIPO_WIDTH, JLJLUP_LIPO_THICKNESS)
        .edges("|Z")
        .fillet(2.5)
    )
    assy = cq.Assembly(name="n203_jljlup_lipo_2000mah_approx")
    assy.add(pouch, name="protected_lipo_pouch_52x34x10", color=cq.Color(0.72, 0.72, 0.70))
    return Model(
        "n203_jljlup_lipo_2000mah_approx",
        "JLJLUP 3.7 V 2000 mAh protected LiPo body; PH2.0 lead and connector are omitted",
        assy,
        source_accuracy="seller-listed 52 x 34 x 10 mm body envelope; corner radii approximate",
        keepout_xy_per_side=1.50,
        keepout_z_below=1.00,
        keepout_z_above=2.00,
    )


def build_dfrobot_c4001() -> Model:
    mounting_holes = [
        (-C4001_MOUNTING_HOLE_SPACING / 2.0, -13.0),
        (C4001_MOUNTING_HOLE_SPACING / 2.0, -13.0),
    ]
    pcb = rounded_pcb_with_holes(
        C4001_PCB_LENGTH,
        C4001_PCB_WIDTH,
        C4001_PCB_THICKNESS_APPROX,
        2.00,
        mounting_holes,
        C4001_MOUNTING_HOLE_DIAMETER,
    )
    signal_holes = [((index - 2.0) * 2.54, -13.0) for index in range(5)]
    pcb = (
        pcb.faces(">Z")
        .workplane()
        .pushPoints(signal_holes)
        .hole(1.00, C4001_PCB_THICKNESS_APPROX)
    )
    radar_soc = box(5.2, 5.2, 1.00, center_y=1.5, bottom_z=C4001_PCB_THICKNESS_APPROX)
    support_zone = box(11.0, 5.0, 1.60, center_y=-6.0, bottom_z=C4001_PCB_THICKNESS_APPROX)
    antenna_patches = [
        box(4.0, 4.0, 0.08, center_x=x, center_y=y, bottom_z=C4001_PCB_THICKNESS_APPROX)
        for x in (-8.0, 8.0)
        for y in (9.0, 2.0, -5.0)
    ]

    assy = cq.Assembly(name="n203_dfrobot_c4001_sen0609_approx")
    assy.add(pcb, name="pcb_26x30_holes_from_dfrobot", color=cq.Color(0.12, 0.25, 0.20))
    assy.add(radar_soc, name="radar_soc_envelope", color=cq.Color(0.12, 0.12, 0.12))
    assy.add(support_zone, name="support_components_envelope", color=cq.Color(0.12, 0.12, 0.12))
    for index, patch in enumerate(antenna_patches, start=1):
        assy.add(patch, name=f"antenna_patch_{index}_schematic", color=cq.Color(0.72, 0.43, 0.13))
    return Model(
        "n203_dfrobot_c4001_sen0609_approx",
        "DFRobot C4001 SEN0609 with published 26 x 30 mm outline and mounting pattern",
        assy,
        source_accuracy="official X/Y drawing; PCB thickness and component heights approximate",
    )


def build_adafruit_bq24074() -> Model:
    mounting_holes = [
        (x, y)
        for x in (-BQ24074_MOUNTING_SPACING_X / 2.0, BQ24074_MOUNTING_SPACING_X / 2.0)
        for y in (-BQ24074_MOUNTING_SPACING_Y / 2.0, BQ24074_MOUNTING_SPACING_Y / 2.0)
    ]
    pcb = rounded_pcb_with_holes(
        BQ24074_PCB_LENGTH,
        BQ24074_PCB_WIDTH,
        BQ24074_PCB_THICKNESS_APPROX,
        BQ24074_CORNER_RADIUS,
        mounting_holes,
        BQ24074_MOUNTING_HOLE_DIAMETER,
    )
    header_holes = [((index - 5.0) * 2.54, -13.97) for index in range(11)]
    pcb = (
        pcb.faces(">Z")
        .workplane()
        .pushPoints(header_holes)
        .hole(1.00, BQ24074_PCB_THICKNESS_APPROX)
    )

    # Connector X/Y extents are taken from the package outlines and placements
    # in Adafruit's Eagle board (origin moved to the PCB centre): the USB-C
    # shell overhangs the -X edge by 1.02 mm, the barrel jack by 3.22 mm, and
    # the side-entry JST-PH bodies stop 0.56 mm inside the +X edge.
    usb_c = box(7.35, 8.94, 3.20, center_x=-16.39, center_y=-6.35, bottom_z=BQ24074_PCB_THICKNESS_APPROX)
    dc_jack = box(14.00, 9.00, 11.00, center_x=-15.27, center_y=5.21, bottom_z=BQ24074_PCB_THICKNESS_APPROX)
    battery_jst = box(7.50, 8.00, 5.00, center_x=14.74, center_y=-5.72, bottom_z=BQ24074_PCB_THICKNESS_APPROX)
    system_jst = box(7.50, 8.00, 5.00, center_x=14.74, center_y=5.72, bottom_z=BQ24074_PCB_THICKNESS_APPROX)
    charger_zone = box(19.0, 14.0, 2.20, center_x=2.0, center_y=2.0, bottom_z=BQ24074_PCB_THICKNESS_APPROX)

    assy = cq.Assembly(name="n203_adafruit_bq24074_4755_approx")
    assy.add(pcb, name="pcb_outline_and_holes_from_adafruit_eagle", color=cq.Color(0.10, 0.28, 0.50))
    assy.add(usb_c, name="usb_c_envelope_approx", color=cq.Color(0.65, 0.67, 0.69))
    assy.add(dc_jack, name="dc_barrel_jack_envelope_approx", color=cq.Color(0.10, 0.10, 0.10))
    assy.add(battery_jst, name="battery_jst_ph_envelope_approx", color=cq.Color(0.88, 0.88, 0.84))
    assy.add(system_jst, name="system_jst_ph_envelope_approx", color=cq.Color(0.88, 0.88, 0.84))
    assy.add(charger_zone, name="charger_components_envelope", color=cq.Color(0.14, 0.14, 0.14))
    return Model(
        "n203_adafruit_bq24074_4755_approx",
        "Adafruit BQ24074 item 4755 with PCB outline and holes derived from official Eagle files",
        assy,
        source_accuracy="official PCB outline, hole pattern, and connector X/Y; component heights approximate",
    )


MODEL_BUILDERS: tuple[Callable[[], Model], ...] = (
    build_display,
    build_s3,
    build_c3,
    build_rd03d,
    build_pololu_s13v20f5,
    build_gy521_mpu6050,
    build_jljlup_lipo,
    build_dfrobot_c4001,
    build_adafruit_bq24074,
)


LOCAL_ONLY_BUILDERS: tuple[Callable[[], Model | None], ...] = (
    build_pololu_s13v20f5_official,
)


def bounds(shape: cq.Shape) -> dict[str, float]:
    bb = shape.BoundingBox()
    return {
        "min_x": round(bb.xmin, 3),
        "max_x": round(bb.xmax, 3),
        "min_y": round(bb.ymin, 3),
        "max_y": round(bb.ymax, 3),
        "min_z": round(bb.zmin, 3),
        "max_z": round(bb.zmax, 3),
        "size_x": round(bb.xlen, 3),
        "size_y": round(bb.ylen, 3),
        "size_z": round(bb.zlen, 3),
    }


def make_keepout(shape: cq.Shape, model: Model) -> cq.Workplane:
    bb = shape.BoundingBox()
    length = bb.xlen + (2.0 * model.keepout_xy_per_side)
    width = bb.ylen + (2.0 * model.keepout_xy_per_side)
    min_z = bb.zmin - model.keepout_z_below
    height = bb.zlen + model.keepout_z_below + model.keepout_z_above
    return box(
        length,
        width,
        height,
        center_x=(bb.xmin + bb.xmax) / 2.0,
        center_y=(bb.ymin + bb.ymax) / 2.0,
        bottom_z=min_z,
    )


def export_model(model: Model, output_dir: Path) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    step_path = output_dir / f"{model.slug}.step"
    stl_path = output_dir / f"{model.slug}.stl"
    keepout_path = output_dir / f"{model.slug}_keepout.step"

    model.assembly.save(str(step_path))
    cq.exporters.export(model.compound, str(stl_path), tolerance=0.05, angularTolerance=0.1)

    keepout = make_keepout(model.compound, model)
    cq.exporters.export(keepout, str(keepout_path))

    # Re-open both STEP files through OpenCascade. This catches truncated or
    # invalid exports before presenting the files as Fusion-ready artifacts.
    imported_model = cq.importers.importStep(str(step_path)).val()
    imported_keepout = cq.importers.importStep(str(keepout_path)).val()
    if imported_model.isNull() or imported_keepout.isNull():
        raise RuntimeError(f"STEP validation failed for {model.slug}")

    return {
        "description": model.description,
        "source_accuracy": model.source_accuracy,
        "step": step_path.name,
        "stl": stl_path.name,
        "keepout_step": keepout_path.name,
        "model_bounds_mm": bounds(model.compound),
        "keepout_bounds_mm": bounds(keepout.val()),
        "keepout_clearance_mm": {
            "xy_per_side": model.keepout_xy_per_side,
            "z_below": model.keepout_z_below,
            "z_above": model.keepout_z_above,
        },
        "step_validation": "passed",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "models",
        help="directory for generated STEP/STL files",
    )
    args = parser.parse_args()

    manifest: dict[str, object] = {
        "units": "mm",
        "warning": (
            "Enclosure-layout references: approximate geometry; "
            "verify against physical parts before printing."
        ),
        "clearance": {
            "xy_per_side": XY_CLEARANCE_PER_SIDE,
            "z_below": Z_CLEARANCE_BELOW,
            "z_above": Z_CLEARANCE_ABOVE,
        },
        "models": {},
    }

    for builder in MODEL_BUILDERS:
        model = builder()
        record = export_model(model, args.output)
        manifest["models"][model.slug] = record
        size = record["model_bounds_mm"]
        print(
            f"{model.slug}: "
            f"{size['size_x']:.3f} x {size['size_y']:.3f} x {size['size_z']:.3f} mm"
        )

    # Local extras: exported if their source file is present, never listed in
    # the manifest, and ignored by git.
    for optional in LOCAL_ONLY_BUILDERS:
        model = optional()
        if model is not None:
            export_model(model, args.output)
            print(f"{model.slug}: exported for local use only")

    manifest_path = args.output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {manifest_path}")


if __name__ == "__main__":
    main()

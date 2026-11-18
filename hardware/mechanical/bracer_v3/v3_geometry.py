"""Geometry for the N203 right-arm bracer, version 3.

Version 3 keeps the version 2 layout but is drawn for a loosely tuned
0.4 mm printer: 5 mm walls, 4 mm floors and lids, no free-standing post or
peg, half a millimetre of clearance around every board, and every screw
runs through a clearance hole into a captive M3 nut instead of cutting a
thread in the plastic.

Printed parts:

* ``saddle``        arm brace with a flat mounting deck and four strap slots
* ``tray``          one electronics chassis that screws onto the deck
* ``bezel``         flat display frame; the display is clamped between it and
                    four corbels in the tray by four bolts and nuts
* ``power_cover``   flat lid over the battery / charger bay
* ``s3_keeper`` and ``c3_keeper``  bars that hold the two hole-less MCU boards
* ``test_coupon``   small test print of the nut slot, nut pocket, and radar slot

Three coordinate frames are used.

Bracer frame (the assembly frame, identical to the earlier review model):
    X runs from the wrist toward the elbow, Z points away from the dorsal
    forearm, and Y completes the right-handed set.  Looking at the screen from
    the elbow end, +Y is the viewer's RIGHT.  On a right arm held palm-down
    that is the little-finger side, so the thumb side is -Y.

Tray frame "T" (u, v, w):
    The deck is tilted ``DECK_TILT_DEG`` so the electronics follow the top of
    the forearm instead of standing on stilts above the wrist.  u runs along
    the deck toward the elbow, v equals bracer Y, and w is normal to the deck.
    The tray is modelled, and printed, in this frame with its floor on w = 0.

Display frame "D" (p, q, r):
    Origin on the bezel seat plane at the display PCB centre.  p is the
    display's long axis (the component model's +X, which points to -v so the
    header lands on the viewer's right), q runs down the slope, and r is the
    outward screen normal.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import cadquery as cq


HERE = Path(__file__).resolve().parent
MECHANICAL_ROOT = HERE.parent
COMPONENT_MODELS = MECHANICAL_ROOT / "models"

sys.path.insert(0, str(MECHANICAL_ROOT))
import arm_fit  # noqa: E402  (shared by every bracer version)

OUTPUT_DIR = arm_fit.output_dir(HERE)

Vec = cq.Vector


# --------------------------------------------------------------------------
# Arm fit: (distance from the wrist end, width, height) of the forearm at each
# station.  ../arm_fit.py holds the reference arm and explains how to use your
# own measurements.
ARM_STATIONS = arm_fit.ARM_STATIONS
BRACER_LENGTH = ARM_STATIONS[-1][0]
FIT_ALLOWANCE_PER_SIDE = 2.00
SHELL_THICKNESS = 4.00
OPEN_UNDERSIDE_Z = -10.00

# Strap slots are real through-cuts in the shell wall, one per side at each
# station, for 20 mm hook-and-loop or webbing straps.
STRAP_WIDTH = 20.00
STRAP_STATIONS = (24.00, 123.00)
STRAP_SLOT_LENGTH = 22.00
STRAP_SLOT_HEIGHT = 6.00
STRAP_SLOT_CENTER_Z = -2.00
STRAP_BAND_THICKNESS = 6.50
STRAP_BAND_LENGTH = 34.00
STRAP_BAND_TOP_Z = 9.00

# Mounting deck.  A flat pad on top of the shell, tilted to follow the dorsal
# ridge, solid down to the arm cavity so the saddle prints deck-down without
# supports and the tray screws have material to bite into.
DECK_TILT_DEG = 10.00
DECK_MIN_GAP = 5.00
# The deck block's straight edges stay this far outside the line where the
# shell surface is 45 degrees to the deck (see deck_half_widths), so the shell
# beyond the block never overhangs by more than about 45 degrees.
DECK_EDGE_MARGIN = 1.50
DECK_FILL_DEPTH = 30.00
TRAY_X_START = 12.00

# --------------------------------------------------------------------------
# Fasteners and print clearances
# --------------------------------------------------------------------------
# Nothing is self-tapped.  Printed holes come out small and uneven on a loose
# printer, and a thread cut into a boss pulls its layers apart.  Every bolt
# passes through clearance holes and clamps the plastic between its head and
# a steel nut.
M3_CLEARANCE_D = 3.80  # prints near 3.4; run a 3.5 mm drill through if tight
M3_HEAD_D = 5.60
M3_HEAD_H = 2.20
M3_NUT_AF = 5.50
M3_NUT_T = 2.40
NUT_SLOT_W = 6.20  # wider than the flats, narrower than the 6.35 mm corners
NUT_SLOT_H = 3.20
NUT_SLOT_END = 3.50  # bolt axis to the closed end of a side slot
NUT_POCKET_AF = 6.00  # drop-in hexagonal pocket
SADDLE_NUT_SEAT = 5.00  # deck plastic between the tray floor and the nut
SADDLE_NUT_RECESS = 0.50  # nut and bolt tip stay this far outside the arm cavity

# Rules the checks enforce on every printed part.
MIN_WALL = 2.40  # thinnest wall between two opposite faces (six 0.4 mm lines)
MIN_ISLAND_AREA = 50.00  # smallest free-standing cross-section, mm^2
BOARD_FIT = 0.50  # clearance per side around a board

# --------------------------------------------------------------------------
# Tray envelope
# --------------------------------------------------------------------------
WALL = 5.00
FLOOR = 4.00
TRAY_INNER_HALF_W = 50.00
TRAY_OUTER_HALF_W = TRAY_INNER_HALF_W + WALL
V_IN = TRAY_INNER_HALF_W
V_OUT = TRAY_OUTER_HALF_W

SCREEN_ANGLE_FROM_DECK_DEG = 45.00
POWER_BAY_INNER_H = 16.00
DIVIDER_LEDGE = 2.00
DIVIDER_BACK = 3.00  # divider thickness behind the bezel's lower edge

# Display (LCDWiki MSP3520 drawing).
DISPLAY_PCB_LENGTH = 98.00
DISPLAY_PCB_WIDTH = 56.34
DISPLAY_PCB_T = 1.60
DISPLAY_LCD_LENGTH = 84.96
DISPLAY_LCD_WIDTH = 55.50
DISPLAY_FRONT_STACK = 4.20  # LCD + touch panel above the PCB front
DISPLAY_HOLES = tuple(
    (x, y) for x in (-46.00, 46.00) for y in (-24.75, 24.75)
)
DISPLAY_HOLE_PAD_D = 4.70
DISPLAY_HEADER_P = -47.00  # header row, 2.00 mm inside the short edge
DISPLAY_HEADER_HALF_SPAN = 7.0 * 2.54  # 14 positions
DISPLAY_LOW_MARGIN = 5.00  # slope distance, bezel lower edge to PCB lower edge
DISPLAY_TOP_MARGIN = 14.00  # slope distance, PCB upper edge to inner apex

BEZEL_T = 5.00
BEZEL_PAD_H = 0.00  # the bezel's flat back bears on the PCB end strips; no pads
BEZEL_WINDOW_CLEARANCE = 0.70
CORBEL_T = 11.00
CORBEL_INBOARD = 6.50  # corbel reach past the hole, toward the screen centre
CORBEL_Q_REACH = 5.70  # corbel reach past the hole, along the slope
DISPLAY_NUT_COVER = 5.00  # plastic between the display PCB and its nut
# Rear microSD socket envelope from the display model (p0, p1, q0, q1, height).
DISPLAY_SD_ENVELOPE = (16.50, 42.50, 8.50, 25.50, 2.20)

# Pin-header wiring allowance (2.54 mm header plastic + 14 mm jumper housing).
HEADER_PLASTIC_H = 2.54
DUPONT_HOUSING_L = 14.00

# MCU boards sit component-side down so their header pins point up at the
# service opening.  BOARD_REST_H is the gap between floor and component face.
BOARD_REST_H = 5.50
BOARD_T = 1.60
USB_C_SHELL_H = 3.20
KEEPER_T = 5.00
KEEPER_BLOCK_HALF = 6.50
KEEPER_NUT_COVER = 3.50

S3_LEN = 63.50
S3_WID = 27.94
S3_ROW_SPACING = 25.40
S3_VC = 31.00
S3_USB_PORT_OFFSETS = (-5.00, 5.00)
S3_KEEPER_W = 12.00
S3_MORTISE_DEPTH = 2.50
S3_TONGUE_STEP = 0.80
S3_RAIL_T = 6.00
S3_RAIL_HALF = 28.00
S3_TIE_OFFSETS = (-15.50, 0.00, 15.50)  # zip-tie tunnels through the lane-side rail

C3_LEN = 22.00
C3_WID = 18.00
C3_ROW_SPACING = 15.24
C3_VC = 25.50
C3_KEEPER_W = 10.00
C3_RAIL_T = 4.00
C3_STUB_GAP = 2.20
# The C3 stands off the leaning distal wall so a jumper on its first pins
# clears the wall higher up.
C3_WALL_GAP = 3.50
C3_TONGUE_STEP = 1.00

RADAR_LEN = 44.00
RADAR_WID = 15.00
RADAR_PCB_T = 1.20
RADAR_LIP_T = 2.60
RADAR_SLOT_T = 1.90
RADAR_SLOT_FIT = 0.50
RADAR_TOP_Y = -0.60
RADAR_EDGE_GRIP = 1.00
RADAR_RAIL_GRIP = 1.00
RADAR_RAIL_PROUD = 2.00

CHARGER_LEN = 38.10
CHARGER_WID = 33.02
CHARGER_STANDOFF_H = 2.50
CHARGER_HOLE_DX = 33.02 / 2.0
CHARGER_HOLE_DY = 27.94 / 2.0
CHARGER_LED_X = 13.34  # D1/D3 in the Adafruit board file, measured from PCB centre

BATTERY_LEN = 52.00
BATTERY_WID = 34.00
BATTERY_T = 10.00
BATTERY_CLEARANCE = 1.50
BATTERY_FOAM_GAP = 2.00

REG_HALF_X = 4.45
REG_HALF_Y = 6.03
REG_BOTTOM_Z = -1.35
REG_TOP_Z = 4.02

COVER_T = 4.00
COVER_GAP = 0.40


# --------------------------------------------------------------------------
# Derived layout
# --------------------------------------------------------------------------
ALPHA = math.radians(SCREEN_ANGLE_FROM_DECK_DEG)
DELTA = math.radians(DECK_TILT_DEG)
SIN_A, COS_A, TAN_A = math.sin(ALPHA), math.cos(ALPHA), math.tan(ALPHA)
SIN_D, COS_D, TAN_D = math.sin(DELTA), math.cos(DELTA), math.tan(DELTA)
# The screen tilts about the forearm axis and faces the thumb side (-v).  Its
# low edge is toward -v and its high edge meets the vertical back wall at +v.
APEX_ANGLE = math.pi / 2.0 - ALPHA

S_PCB_LOW = DISPLAY_LOW_MARGIN
S_PCB_HIGH = S_PCB_LOW + DISPLAY_PCB_WIDTH
S_PCB_CENTER = S_PCB_LOW + DISPLAY_PCB_WIDTH / 2.0
S_INNER_APEX = S_PCB_HIGH + DISPLAY_TOP_MARGIN
S_OUTER_APEX = S_INNER_APEX + WALL / math.sin(APEX_ANGLE)

RIM_H = FLOOR + POWER_BAY_INNER_H
APEX_W = RIM_H + S_OUTER_APEX * SIN_A
SLOPE_V = V_OUT - S_OUTER_APEX * COS_A  # v where the bezel seat meets the flat rim
DIVIDER_V0 = SLOPE_V - DIVIDER_LEDGE  # power-bay face
DIVIDER_V1 = SLOPE_V + DIVIDER_BACK  # logic-bay face

PCB_FRONT_R = -BEZEL_PAD_H
PCB_BACK_R = -(BEZEL_PAD_H + DISPLAY_PCB_T)

BOARD_BOTTOM_W = FLOOR + BOARD_REST_H
BOARD_TOP_W = BOARD_BOTTOM_W + BOARD_T
REST_PAD_TOP_W = BOARD_BOTTOM_W - USB_C_SHELL_H
KEEPER_TOP_W = BOARD_TOP_W + KEEPER_T
COVER_TOP_W = RIM_H + COVER_T


def wall_outer_u(w: float) -> float:
    """u of the distal wall's outer face at height w (tray frame).

    The distal wall leans DECK_TILT_DEG so it is square to the forearm axis
    and the radar looks straight along the arm.
    """
    return w * TAN_D


def wall_inner_u(w: float) -> float:
    """u of the distal wall's inner face at height w (tray frame)."""
    return WALL / COS_D + w * TAN_D


def rim_w(v: float) -> float:
    """Height of the bezel seat plane at lateral position v."""
    return RIM_H + (v - SLOPE_V) * TAN_A


def display_clear_w(v: float) -> float:
    """Height below which nothing on the back of the display can reach.

    Uses the 3.00 mm maximum rear component height from the LCDWiki drawing.
    """
    return rim_w(v) - (BEZEL_PAD_H + DISPLAY_PCB_T + 3.00) / COS_A


# ---- power bay (flat-covered strip along the thumb side) -------------------
BATTERY_V0 = -V_IN  # pocket runs from the -v wall ...
BATTERY_V1 = BATTERY_V0 + BATTERY_WID + 2.0 * BATTERY_CLEARANCE  # ... to a rib
BATTERY_VC = (BATTERY_V0 + BATTERY_V1) / 2.0
BATTERY_U0 = wall_inner_u(FLOOR + BATTERY_T) + 1.20
BATTERY_U1 = BATTERY_U0 + BATTERY_LEN
BATTERY_UC = (BATTERY_U0 + BATTERY_U1) / 2.0
# The pouch stops against a wall here.  Behind that wall, in order: the
# regulator pocket, a fence shared with the charger, then the charger.
BATTERY_STOP_U = BATTERY_U1 + BATTERY_CLEARANCE
STOP_WALL_T = 3.00
FENCE_T = 3.50
CHARGER_FIT = 0.60
REG_FIT = 0.50
REG_POCKET_U0 = BATTERY_STOP_U + STOP_WALL_T
REG_UC = REG_POCKET_U0 + REG_FIT + REG_HALF_X

CHARGER_U0 = REG_UC + REG_HALF_X + REG_FIT + FENCE_T + CHARGER_FIT
CHARGER_UC = CHARGER_U0 + CHARGER_WID / 2.0
CHARGER_U1 = CHARGER_U0 + CHARGER_WID
CHARGER_VC = -V_IN + 0.50 + CHARGER_LEN / 2.0
CHARGER_PCB_BOTTOM_W = FLOOR + CHARGER_STANDOFF_H
CHARGER_PCB_TOP_W = CHARGER_PCB_BOTTOM_W + 1.60
CHARGER_JST_FACE_V = CHARGER_VC + 18.49
FENCE_TOP_W = CHARGER_PCB_TOP_W + 0.40
FENCE_V1 = -16.00

CORNER_BLOCK_L = 13.00
TRAY_L = CHARGER_U1 + CHARGER_FIT + CORNER_BLOCK_L + WALL
U_IN_END = TRAY_L - WALL  # inner face of the proximal wall

# Cover bolts.  Each drops into a block that is part of a wall and has a side
# slot for an M3 nut.  Tuples are (u, v, side the nut slides in from: +1 = +v).
COVER_NUT_COVER = 5.00  # plastic between the cover and the nut
COVER_BLOCK_DEPTH = 9.00
COVER_BOLT_V_WALL = -V_IN + 4.10
COVER_BOLT_V_DIVIDER = DIVIDER_V0 - 3.30
BLOCK_C_V1 = -V_IN + 8.60
BLOCK_C_UC = (REG_POCKET_U0 - 0.50 + CHARGER_U0 - CHARGER_FIT) / 2.0
BLOCK_D_UC = wall_inner_u(RIM_H) + 7.00
BLOCK_D_U1 = BLOCK_D_UC + 7.00
COVER_BOSSES = (
    (U_IN_END - CORNER_BLOCK_L / 2.0, COVER_BOLT_V_WALL, 1.0),
    (U_IN_END - CORNER_BLOCK_L / 2.0, COVER_BOLT_V_DIVIDER, -1.0),
    (BLOCK_C_UC, COVER_BOLT_V_WALL, 1.0),
    (BLOCK_D_UC, COVER_BOLT_V_DIVIDER, -1.0),
)
# Blocks as (u0, u1, v0, v1): proximal corners A and B, C beside the
# regulator, D beside the battery at the distal end.
COVER_BLOCKS = (
    (CHARGER_U1 + CHARGER_FIT, U_IN_END + 0.50, -V_IN - 0.50, -V_IN + COVER_BLOCK_DEPTH),
    (CHARGER_U1 + CHARGER_FIT, U_IN_END + 0.50, DIVIDER_V0 - COVER_BLOCK_DEPTH, DIVIDER_V0 + 0.50),
    (REG_POCKET_U0 - 0.50, CHARGER_U0 - CHARGER_FIT, -V_IN - 0.50, BLOCK_C_V1),
    (0.00, BLOCK_D_U1, BATTERY_V1, DIVIDER_V0 + 0.50),
)

# The regulator sits in a pocket: battery stop wall, block C, and the fence.
REG_VC = BLOCK_C_V1 + REG_FIT + REG_HALF_Y
REG_V1_FACE = REG_VC + REG_HALF_Y + REG_FIT
REG_Z_OFFSET = FLOOR - REG_BOTTOM_Z
REG_TOP_W = REG_Z_OFFSET + REG_TOP_Z

# The charger's JST plugs reach through this notch in the divider; it is also
# where power crosses into the logic bay.
JST_NOTCH_U = (CHARGER_UC - 16.00, CHARGER_UC + 9.50)
JST_NOTCH_BOTTOM_W = CHARGER_PCB_TOP_W - 1.00

# ---- logic bay (under the display) -----------------------------------------
DISPLAY_U1 = U_IN_END - 0.50
DISPLAY_U0 = DISPLAY_U1 - DISPLAY_PCB_LENGTH
DISPLAY_UC = (DISPLAY_U0 + DISPLAY_U1) / 2.0

S3_U_USB_END = U_IN_END - BOARD_FIT  # PCB end against the proximal wall
S3_UC = S3_U_USB_END - S3_LEN / 2.0
S3_U_FAR_END = S3_U_USB_END - S3_LEN
S3_BOSS_U = S3_U_FAR_END - BOARD_FIT - KEEPER_BLOCK_HALF

C3_U_USB_END = wall_inner_u(BOARD_TOP_W) + C3_WALL_GAP
C3_UC = C3_U_USB_END + C3_LEN / 2.0
C3_U_FAR_END = C3_U_USB_END + C3_LEN
C3_BOSS_U = C3_U_FAR_END + BOARD_FIT + KEEPER_BLOCK_HALF
C3_STOP_U1 = C3_U_USB_END - BOARD_FIT
C3_GOAL_BOTTOM_W = KEEPER_TOP_W + 0.10
C3_GOAL_TOP_W = C3_GOAL_BOTTOM_W + 3.50
C3_TONGUE_U0 = wall_inner_u(KEEPER_TOP_W) + 0.50

# Radar slot in the back wall, centred behind the screen, so the radar looks
# out of the back of the display, square to the forearm.  The slot leans
# DECK_TILT_DEG within the wall so the board's long axis stays square to the
# forearm axis (the datasheet's "up").  Local y = 0 is where the board's higher
# top corner would meet the bezel seat.
RADAR_U = DISPLAY_UC
RADAR_PCB_BACK_Z = RADAR_LIP_T + RADAR_PCB_T
RADAR_REF_W = rim_w(V_OUT - RADAR_PCB_BACK_Z) - RADAR_WID / 2.0 * SIN_D
RADAR_CENTER_Y = RADAR_TOP_Y - RADAR_LEN / 2.0
RADAR_BOTTOM_Y = RADAR_TOP_Y - RADAR_LEN

# Tray-to-saddle bolts (u, v, parallel_to_distal_wall).  Three run down the
# wiring lane of the logic bay and two sit in the power bay; none is under a
# board, and all are far enough from the leaning distal wall for a straight
# driver.  The third field is kept for a bolt that has to lean with that wall.
LANE_V = 6.00
TRAY_SCREWS = (
    (17.00, LANE_V, False),
    (61.00, LANE_V, False),
    (105.00, LANE_V, False),
    (REG_POCKET_U0 + 5.50, -20.00, False),
    (U_IN_END - CORNER_BLOCK_L / 2.0, -27.00, False),
)
TRAY_SCREW_SPOTFACE_D = 6.60
TRAY_SCREW_SPOTFACE_DEPTH = 0.60


# --------------------------------------------------------------------------
# Small solid helpers (all return cq.Shape)
# --------------------------------------------------------------------------
def box3(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Shape:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
        .translate((x0, y0, z0))
        .val()
    )


def vcyl(diameter: float, x: float, y: float, z0: float, z1: float) -> cq.Shape:
    return cq.Solid.makeCylinder(diameter / 2.0, z1 - z0, Vec(x, y, z0), Vec(0, 0, 1))


def prism_uw(points: list[tuple[float, float]], v0: float, v1: float) -> cq.Shape:
    """Extrude a polygon drawn in the (u, w) side view across v0..v1."""
    return (
        cq.Workplane("XZ", origin=(0, v1, 0))
        .polyline(points)
        .close()
        .extrude(v1 - v0)
        .val()
    )


def prism_uv(points: list[tuple[float, float]], w0: float, w1: float) -> cq.Shape:
    return (
        cq.Workplane("XY", origin=(0, 0, w0))
        .polyline(points)
        .close()
        .extrude(w1 - w0)
        .val()
    )


def prism_yz(points: list[tuple[float, float]], x0: float, x1: float) -> cq.Shape:
    return (
        cq.Workplane("YZ", origin=(x0, 0, 0))
        .polyline(points)
        .close()
        .extrude(x1 - x0)
        .val()
    )


def hex_prism(across_flats: float, z0: float, z1: float) -> cq.Shape:
    """Hexagonal prism on the local z axis; one pair of flats faces +/-y."""
    across_corners = across_flats / math.cos(math.radians(30.0))
    return (
        cq.Workplane("XY")
        .polygon(6, across_corners)
        .extrude(z1 - z0)
        .val()
        .translate(Vec(0.0, 0.0, z0))
    )


def fuse_all(base: cq.Shape, parts: list[cq.Shape]) -> cq.Shape:
    return base.fuse(*parts) if parts else base


def cut_all(base: cq.Shape, parts: list[cq.Shape]) -> cq.Shape:
    return base.cut(*parts) if parts else base


def cleaned(shape: cq.Shape) -> cq.Shape:
    try:
        return shape.clean()
    except Exception:  # pragma: no cover - cosmetic only
        return shape


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


@dataclass(frozen=True)
class Frame:
    """A right-handed local frame expressed in its parent frame."""

    origin: tuple[float, float, float]
    x_dir: tuple[float, float, float]
    z_dir: tuple[float, float, float]

    @property
    def y_dir(self) -> tuple[float, float, float]:
        return _cross(self.z_dir, self.x_dir)

    @property
    def location(self) -> cq.Location:
        return cq.Location(cq.Plane(origin=self.origin, xDir=self.x_dir, normal=self.z_dir))

    def place(self, shape: cq.Shape) -> cq.Shape:
        """Map a shape drawn in local coordinates into the parent frame."""
        return shape.moved(self.location)

    def unplace(self, shape: cq.Shape) -> cq.Shape:
        return shape.moved(self.location.inverse)

    def point(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        yd = self.y_dir
        return tuple(
            self.origin[i] + x * self.x_dir[i] + y * yd[i] + z * self.z_dir[i]
            for i in range(3)
        )

    def direction(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        yd = self.y_dir
        return tuple(x * self.x_dir[i] + y * yd[i] + z * self.z_dir[i] for i in range(3))


# Display frame in the tray frame.  p runs toward the wrist so the header
# (at -p) is toward the elbow, which is the viewer's right when the screen is
# read from the thumb side.  q runs down the slope, r is the screen normal.
D_FRAME = Frame(
    origin=(DISPLAY_UC, SLOPE_V + S_PCB_CENTER * COS_A, RIM_H + S_PCB_CENTER * SIN_A),
    x_dir=(-1.0, 0.0, 0.0),
    z_dir=(0.0, -SIN_A, COS_A),
)

# Radar frame in the tray frame: origin on the back wall's outer face, local z
# through the wall into the cavity, local y along the board's long axis (the
# forearm's dorsal direction), local x across the board.
W_FRAME = Frame(
    origin=(RADAR_U, V_OUT, RADAR_REF_W),
    x_dir=(COS_D, 0.0, -SIN_D),
    z_dir=(0.0, -1.0, 0.0),
)

# Both MCU boards sit component side down (model +Z toward the floor).
S3_FRAME = Frame(origin=(S3_UC, S3_VC, BOARD_TOP_W), x_dir=(-1.0, 0.0, 0.0), z_dir=(0.0, 0.0, -1.0))
C3_FRAME = Frame(origin=(C3_UC, C3_VC, BOARD_TOP_W), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, 0.0, -1.0))
RADAR_LOCAL_FRAME = Frame(
    origin=(0.0, RADAR_CENTER_Y, RADAR_PCB_BACK_Z),
    x_dir=(0.0, 1.0, 0.0),
    z_dir=(0.0, 0.0, -1.0),
)
CHARGER_FRAME = Frame(
    origin=(CHARGER_UC, CHARGER_VC, CHARGER_PCB_BOTTOM_W),
    x_dir=(0.0, 1.0, 0.0),
    z_dir=(0.0, 0.0, 1.0),
)


def slope_to_q(s: float) -> float:
    """Slope distance from the bezel's lower edge -> display-frame q."""
    return S_PCB_CENTER - s


def tray_screw_frame(u: float, v: float, tilted: bool) -> Frame:
    """Frame whose +z is the screw axis (toward the head), origin on the seat.

    For a tilted screw the seat is a small spot-face square to the axis.
    """
    if not tilted:
        return Frame(origin=(u, v, FLOOR), x_dir=(0.0, 1.0, 0.0), z_dir=(0.0, 0.0, 1.0))
    axis = (SIN_D, 0.0, COS_D)
    depth = TRAY_SCREW_SPOTFACE_DEPTH
    return Frame(
        origin=(u - depth * axis[0], v, FLOOR - depth * axis[2]),
        x_dir=(0.0, 1.0, 0.0),
        z_dir=axis,
    )


def tray_screw_deck_drop(tilted: bool) -> float:
    """Distance along the screw axis from the head seat down to the deck plane."""
    if not tilted:
        return FLOOR
    return FLOOR / COS_D - TRAY_SCREW_SPOTFACE_DEPTH


@dataclass(frozen=True)
class NutSite:
    """A bolt whose M3 nut slides into a side slot of a printed block.

    In the site's local frame +z points at the bolt head, z = 0 is the face
    the clamped part sits on, and the nut slides in from +x.
    """

    name: str
    frames: tuple[Frame, ...]  # innermost first
    cover: float  # plastic between the seating face and the nut
    hole_depth: float  # bolt hole depth below the seating face
    reach: float  # how far the slot is cut toward its open side

    def place(self, shape: cq.Shape) -> cq.Shape:
        for frame in self.frames:
            shape = frame.place(shape)
        return shape

    def direction(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        vector = (x, y, z)
        for frame in self.frames:
            vector = frame.direction(*vector)
        return vector

    @property
    def cut(self) -> cq.Shape:
        hole = vcyl(M3_CLEARANCE_D, 0.0, 0.0, -self.hole_depth, 0.5)
        slot = box3(
            -NUT_SLOT_END,
            self.reach,
            -NUT_SLOT_W / 2.0,
            NUT_SLOT_W / 2.0,
            -self.cover - NUT_SLOT_H,
            -self.cover,
        )
        return self.place(hole.fuse(slot))

    @property
    def nut(self) -> cq.Shape:
        """The nut drawn up against the roof of its slot."""
        return self.place(hex_prism(M3_NUT_AF, -self.cover - M3_NUT_T, -self.cover))


def side_nut_sites() -> list[NutSite]:
    up = (0.0, 0.0, 1.0)
    sites: list[NutSite] = []
    for index, (hole_p, hole_q) in enumerate(DISPLAY_HOLES, start=1):
        inboard = 1.0 if hole_p < 0 else -1.0
        local = Frame(origin=(hole_p, hole_q, PCB_BACK_R), x_dir=(inboard, 0.0, 0.0), z_dir=up)
        # An M3x14 ends flush with the far face of the nut, so the hole stops
        # at the slot and the plastic under the nut stays whole.
        depth = DISPLAY_NUT_COVER + NUT_SLOT_H
        sites.append(NutSite(f"display_nut_{index}", (local, D_FRAME), DISPLAY_NUT_COVER, depth, 12.0))
    for index, (boss_u, boss_v, entry) in enumerate(COVER_BOSSES, start=1):
        local = Frame(origin=(boss_u, boss_v, RIM_H), x_dir=(0.0, entry, 0.0), z_dir=up)
        sites.append(
            NutSite(f"cover_nut_{index}", (local,), COVER_NUT_COVER, COVER_NUT_COVER + NUT_SLOT_H + 1.0, 6.5)
        )
    for name, boss_u, boss_v in (("s3_keeper_nut", S3_BOSS_U, S3_VC), ("c3_keeper_nut", C3_BOSS_U, C3_VC)):
        local = Frame(origin=(boss_u, boss_v, BOARD_TOP_W), x_dir=(0.0, -1.0, 0.0), z_dir=up)
        sites.append(NutSite(name, (local,), KEEPER_NUT_COVER, BOARD_TOP_W - FLOOR + 1.0, 9.0))
    return sites


# --------------------------------------------------------------------------
# Saddle (arm brace)
# --------------------------------------------------------------------------
def interpolate_arm_dimension(x: float, dimension_index: int) -> float:
    for left, right in zip(ARM_STATIONS, ARM_STATIONS[1:]):
        if left[0] <= x <= right[0]:
            ratio = (x - left[0]) / (right[0] - left[0])
            return left[dimension_index] + ratio * (
                right[dimension_index] - left[dimension_index]
            )
    station = ARM_STATIONS[0] if x < ARM_STATIONS[0][0] else ARM_STATIONS[-1]
    return station[dimension_index]


def _ellipse_wire(x: float, width: float, height: float) -> cq.Wire:
    return cq.Wire.makeEllipse(
        width / 2.0,
        height / 2.0,
        center=Vec(x, 0.0, 0.0),
        normal=Vec(1.0, 0.0, 0.0),
        xDir=Vec(0.0, 1.0, 0.0),
    )


def _arm_loft(offset: float, extend: float = 0.0) -> cq.Shape:
    """Smooth loft through the three measured stations, grown by ``offset``.

    Every surface (cavity, shell, strap bands) uses the same three sections, so
    the wall between any two of them has a uniform thickness.  ``extend``
    slides the end sections outward along the local taper so a cutter pokes
    through the end faces instead of sharing them.

    The earlier review shell lofted its cavity through five sections (the end
    ones duplicated) but its outside through three.  That made the wall 6.5 mm
    thick near the wrist and 1.3 mm thin toward the elbow, and squeezed the
    cavity below the measured arm between the first two stations.
    """
    stations = [
        [x, width + 2.0 * offset, height + 2.0 * offset]
        for x, width, height in ARM_STATIONS
    ]
    if extend > 0.0:
        for end, neighbour, sign in ((0, 1, -1.0), (-1, -2, 1.0)):
            span = stations[end][0] - stations[neighbour][0]
            for axis in (1, 2):
                slope = (stations[end][axis] - stations[neighbour][axis]) / span
                stations[end][axis] += slope * sign * extend
            stations[end][0] += sign * extend
    return cq.Solid.makeLoft([_ellipse_wire(*station) for station in stations])


@lru_cache(maxsize=None)
def _outer_loft() -> cq.Shape:
    return _arm_loft(FIT_ALLOWANCE_PER_SIDE + SHELL_THICKNESS)


@lru_cache(maxsize=None)
def _inner_loft() -> cq.Shape:
    return _arm_loft(FIT_ALLOWANCE_PER_SIDE, extend=1.0)


def ridge_height(x: float) -> float:
    """Top (dorsal) height of the outer shell surface at station x."""
    slab = box3(x - 0.05, x + 0.05, -3.0, 3.0, 0.0, 200.0)
    return _outer_loft().intersect(slab).BoundingBox().zmax


@lru_cache(maxsize=None)
def deck_offset() -> float:
    """Deck plane z = x * tan(tilt) + offset, clearing the ridge by DECK_MIN_GAP."""
    samples = [0.5 + i * (BRACER_LENGTH - 1.0) / 24.0 for i in range(25)]
    return max(ridge_height(x) - x * TAN_D for x in samples) + DECK_MIN_GAP


@lru_cache(maxsize=None)
def tray_frame() -> Frame:
    """Tray frame expressed in the bracer frame."""
    return Frame(
        origin=(TRAY_X_START, 0.0, TRAY_X_START * TAN_D + deck_offset()),
        x_dir=(COS_D, 0.0, SIN_D),
        z_dir=(-SIN_D, 0.0, COS_D),
    )


def deck_half_width_needed(x: float) -> float:
    """Lateral distance at which the shell's outer surface is 45 degrees to the deck."""
    a = interpolate_arm_dimension(x, 1) / 2.0 + FIT_ALLOWANCE_PER_SIDE + SHELL_THICKNESS
    b = interpolate_arm_dimension(x, 2) / 2.0 + FIT_ALLOWANCE_PER_SIDE + SHELL_THICKNESS
    return a * a / math.hypot(a, b)


@lru_cache(maxsize=None)
def deck_half_widths() -> tuple[float, float]:
    """Deck block half-widths at the wrist and elbow ends, derived from the arm."""
    distal = deck_half_width_needed(0.0) + DECK_EDGE_MARGIN
    proximal = deck_half_width_needed(BRACER_LENGTH) + DECK_EDGE_MARGIN
    # Widen both ends if the straight edge would cut inside the 45-degree line
    # somewhere between them.
    for step in range(1, 20):
        x = BRACER_LENGTH * step / 20.0
        edge = distal + (proximal - distal) * x / BRACER_LENGTH
        shortfall = deck_half_width_needed(x) + DECK_EDGE_MARGIN - edge
        if shortfall > 0.0:
            distal += shortfall
            proximal += shortfall
    return distal, proximal


def _shell_outer_half_width(x: float, z: float, thickness: float = SHELL_THICKNESS) -> float:
    a = interpolate_arm_dimension(x, 1) / 2.0 + FIT_ALLOWANCE_PER_SIDE + thickness
    b = interpolate_arm_dimension(x, 2) / 2.0 + FIT_ALLOWANCE_PER_SIDE + thickness
    return a * math.sqrt(max(0.0, 1.0 - (z / b) ** 2))


@lru_cache(maxsize=None)
def make_saddle() -> cq.Shape:
    frame = tray_frame()
    # Everything is built solid and the arm cavity is cut once at the end, so no
    # two bodies share the cavity surface on the way.  (Fusing bodies that each
    # carried that surface gave an invalid solid for some arm shapes.)
    inner = _inner_loft()

    # Local thickening around each strap slot, tapered at 45 degrees on its
    # upper edge so it still prints without support when the saddle is built
    # deck-down.
    thick = _arm_loft(FIT_ALLOWANCE_PER_SIDE + STRAP_BAND_THICKNESS)
    bands: list[cq.Shape] = []
    for station in STRAP_STATIONS:
        y_ref = _shell_outer_half_width(station, STRAP_BAND_TOP_Z)
        for side in (-1.0, 1.0):
            profile = [
                (side * (y_ref - 12.0), OPEN_UNDERSIDE_Z - 2.0),
                (side * (y_ref + 14.0), OPEN_UNDERSIDE_Z - 2.0),
                (side * (y_ref + 14.0), STRAP_BAND_TOP_Z - 14.0),
                (side * (y_ref - 12.0), STRAP_BAND_TOP_Z + 12.0),
            ]
            region = prism_yz(
                profile,
                station - STRAP_BAND_LENGTH / 2.0,
                station + STRAP_BAND_LENGTH / 2.0,
            )
            bands.append(thick.intersect(region))

    # Deck block, drawn in the tray frame and filled down to the arm cavity.
    # It spans the whole shell so that, printed deck-down, no part of the
    # shell crown hangs past the end of the block.
    half_distal, half_proximal = deck_half_widths()
    u0 = (0.0 - TRAY_X_START) / COS_D - 12.0
    u1 = (BRACER_LENGTH - TRAY_X_START) / COS_D + 12.0
    block = frame.place(
        prism_uv(
            [
                (u0, -half_distal),
                (u1, -half_proximal),
                (u1, half_proximal),
                (u0, half_distal),
            ],
            -DECK_FILL_DEPTH,
            0.0,
        )
    )

    body = _outer_loft().fuse(block, *bands).cut(inner)

    cuts: list[cq.Shape] = []
    for u, v, tilted in TRAY_SCREWS:
        drop = tray_screw_deck_drop(tilted)
        screw_frame = tray_screw_frame(u, v, tilted)
        seat = -drop - SADDLE_NUT_SEAT
        radius = M3_CLEARANCE_D / 2.0
        for cutter in (
            vcyl(M3_CLEARANCE_D, 0.0, 0.0, seat - 1.0, -drop + 0.5),
            # Drop-in nut pocket, open to the arm side of the brace.
            hex_prism(NUT_POCKET_AF, seat - 45.0, seat),
            # Lead-in on the bed face, where the first layers close holes up.
            cq.Solid.makeCone(radius, radius + 0.9, 0.9, Vec(0.0, 0.0, -drop - 0.8), Vec(0.0, 0.0, 1.0)),
        ):
            cuts.append(frame.place(screw_frame.place(cutter)))
    for station in STRAP_STATIONS:
        cuts.append(
            cq.Workplane("XZ", origin=(station, 80.0, STRAP_SLOT_CENTER_Z))
            .slot2D(STRAP_SLOT_LENGTH, STRAP_SLOT_HEIGHT)
            .extrude(160.0)
            .val()
        )
    body = body.cut(*cuts)

    clip = box3(0.0, BRACER_LENGTH, -80.0, 80.0, OPEN_UNDERSIDE_Z, 140.0)
    return cleaned(body.intersect(clip))


# --------------------------------------------------------------------------
# Tray (electronics chassis)
# --------------------------------------------------------------------------
def tray_side_profile() -> list[tuple[float, float]]:
    """Tray cross-section in (v, w): flat power bay, then the screen wedge."""
    return [
        (-V_OUT, 0.0),
        (-V_OUT, RIM_H),
        (SLOPE_V, RIM_H),
        (V_OUT, APEX_W),
        (V_OUT, 0.0),
    ]


def tray_envelope() -> cq.Shape:
    top = APEX_W + 30.0
    lean = prism_uw(
        [(0.0, 0.0), (wall_outer_u(top), top), (TRAY_L, top), (TRAY_L, 0.0)],
        -V_OUT - 1.0,
        V_OUT + 1.0,
    )
    return prism_yz(tray_side_profile(), -40.0, TRAY_L).intersect(lean)


def _corbel_p_range(hole_p: float, hole_q: float) -> tuple[float, float]:
    """Extent of a display corbel along the screen's long axis."""
    if hole_p < 0:
        # Proximal pair: run into the end wall.
        return hole_p - 6.0, hole_p + CORBEL_INBOARD
    p0, p1 = hole_p - CORBEL_INBOARD, hole_p + 9.0
    sd_p0, sd_p1, sd_q0, sd_q1, _ = DISPLAY_SD_ENVELOPE
    if sd_q0 - 6.0 < hole_q < sd_q1 + 6.0:
        # The microSD socket on the back of the display reaches to 3.5 mm
        # from this hole, so the corbel stops short of it.
        p0 = max(p0, sd_p1 + 0.5)
    return p0, p1


def _display_post_blocks() -> list[cq.Shape]:
    """Four corbels whose top faces carry the display PCB (display frame).

    The upper pair grow out of the back wall and the lower pair out of the
    divider.  Each is wide enough to hold a nut slot beside the bolt hole.
    """
    blocks = []
    q_top_end = slope_to_q(S_OUTER_APEX + 4.0)
    for hole_p, hole_q in DISPLAY_HOLES:
        p0, p1 = _corbel_p_range(hole_p, hole_q)
        if hole_q < 0:
            # Upper pair: a gusset.  Its underside is the plane square to the
            # screen through its lower edge, which runs back to the wall, so
            # every layer of it rests on the wall or on the layer below.  (A
            # constant-thickness corbel here would start printing in mid-air.)
            block = D_FRAME.place(
                box3(p0, p1, q_top_end, hole_q + CORBEL_Q_REACH, PCB_BACK_R - 60.0, PCB_BACK_R)
            )
        else:
            # Lower pair: a constant-thickness corbel that leaves the divider
            # face and rises away from it, clipped so nothing reaches into
            # the power bay.
            block = D_FRAME.place(
                box3(p0, p1, hole_q - CORBEL_Q_REACH, slope_to_q(-20.0), PCB_BACK_R - CORBEL_T, PCB_BACK_R)
            ).intersect(box3(-50.0, 300.0, DIVIDER_V0 + 0.5, 100.0, -10.0, 200.0))
        blocks.append(block)
    return blocks


@lru_cache(maxsize=None)
def make_tray() -> cq.Shape:
    outer = tray_envelope()
    top = APEX_W + 30.0
    cavity = prism_uw(
        [
            (wall_inner_u(FLOOR), FLOOR),
            (wall_inner_u(top), top),
            (U_IN_END, top),
            (U_IN_END, FLOOR),
        ],
        -V_IN,
        V_IN,
    )
    body = outer.cut(cavity)

    adds: list[cq.Shape] = []
    cuts: list[cq.Shape] = []
    base = FLOOR - 0.3

    # ---- divider between the power bay and the logic bay -----------------
    adds.append(box3(0.0, U_IN_END + 0.5, DIVIDER_V0, DIVIDER_V1, base, RIM_H + 12.0))
    cuts.append(
        box3(
            JST_NOTCH_U[0],
            JST_NOTCH_U[1],
            DIVIDER_V0 - 0.5,
            DIVIDER_V1 + 0.5,
            JST_NOTCH_BOTTOM_W,
            RIM_H + 14.0,
        )
    )

    # ---- display corbels (bolt holes and nut slots are cut further down) --
    adds.extend(_display_post_blocks())
    radius = M3_CLEARANCE_D / 2.0
    for hole_p, hole_q in DISPLAY_HOLES:
        # Where a corbel ends less than MIN_WALL from its bolt hole, the hole
        # is opened into a U instead of leaving a sliver beside it.
        p0, _ = _corbel_p_range(hole_p, hole_q)
        if hole_p > 0 and hole_p - radius - p0 < MIN_WALL:
            cuts.append(
                D_FRAME.place(
                    box3(
                        p0 - 1.0,
                        hole_p,
                        hole_q - radius,
                        hole_q + radius,
                        PCB_BACK_R - DISPLAY_NUT_COVER - 0.1,
                        PCB_BACK_R + 0.5,
                    )
                )
            )

    # ---- ESP32-S3 cradle (USB end at the proximal wall, pins up) ---------
    s3_v0 = S3_VC - S3_WID / 2.0 - BOARD_FIT
    s3_v1 = S3_VC + S3_WID / 2.0 + BOARD_FIT
    adds.append(box3(S3_UC - 18.0, S3_UC - 8.0, S3_VC - 6.0, S3_VC + 6.0, base, REST_PAD_TOP_W))
    adds.append(box3(U_IN_END - 6.0, U_IN_END + 0.3, S3_VC - 11.0, S3_VC + 11.0, base, REST_PAD_TOP_W))
    # Guide rails instead of tabs: one fills the gap to the back wall, the
    # other stands on the lane side and doubles as the zip-tie anchor.
    adds.append(box3(S3_UC - S3_RAIL_HALF, S3_UC + S3_RAIL_HALF, s3_v1, V_IN + 0.5, base, BOARD_TOP_W))
    adds.append(box3(S3_UC - S3_RAIL_HALF, S3_UC + S3_RAIL_HALF, s3_v0 - S3_RAIL_T, s3_v0, base, BOARD_TOP_W))
    for offset in S3_TIE_OFFSETS:
        cuts.append(
            box3(
                S3_UC + offset - 3.0,
                S3_UC + offset + 3.0,
                s3_v0 - S3_RAIL_T - 0.5,
                s3_v0 + 0.5,
                FLOOR,
                FLOOR + 3.4,
            )
        )
    # Keeper block beyond the far end of the board; it is also the end stop.
    adds.append(
        box3(
            S3_BOSS_U - KEEPER_BLOCK_HALF,
            S3_BOSS_U + KEEPER_BLOCK_HALF,
            S3_VC - 7.0,
            S3_VC + 7.0,
            base,
            BOARD_TOP_W,
        )
    )
    # One opening for both USB plugs.  The PCB corners stop against the full
    # wall either side of it.
    cuts.append(box3(U_IN_END - 0.3, TRAY_L + 1.0, S3_VC - 12.5, S3_VC + 12.5, FLOOR + 0.2, 12.8))
    # Mortise for the end of the keeper bar.
    cuts.append(
        box3(
            U_IN_END - 0.3,
            U_IN_END + S3_MORTISE_DEPTH,
            S3_VC - S3_KEEPER_W / 2.0 - 0.5,
            S3_VC + S3_KEEPER_W / 2.0 + 0.5,
            BOARD_TOP_W + S3_TONGUE_STEP - 0.3,
            KEEPER_TOP_W + 0.6,
        )
    )

    # ---- ESP32-C3 cradle (USB end toward the distal wall, pins up) -------
    adds.append(
        box3(C3_U_USB_END - 1.7, C3_U_USB_END + 5.5, C3_VC - 5.0, C3_VC + 5.0, base, REST_PAD_TOP_W)
    )
    for side in (-1.0, 1.0):
        edge = C3_VC + side * (C3_WID / 2.0 + BOARD_FIT)
        # Rail beside the board ...
        v0, v1 = sorted((edge, edge + side * C3_RAIL_T))
        adds.append(box3(C3_UC - 5.0, C3_UC + 10.0, v0, v1, base, BOARD_TOP_W))
        # ... with a low step under the board edge, clear of the solder stubs.
        v0, v1 = sorted((edge - side * 3.0, edge + side * 0.5))
        adds.append(box3(C3_UC - 5.0, C3_UC + 10.0, v0, v1, base, BOARD_BOTTOM_W - C3_STUB_GAP))
        # Two buttresses on the distal wall stop the PCB end either side of
        # the USB socket ...
        v0, v1 = sorted((C3_VC + side * 6.8, C3_VC + side * 11.8))
        adds.append(
            prism_uw(
                [
                    (wall_inner_u(base) - 0.4, base),
                    (C3_STOP_U1, base),
                    (C3_STOP_U1, C3_GOAL_TOP_W),
                    (wall_inner_u(C3_GOAL_TOP_W) - 0.4, C3_GOAL_TOP_W),
                ],
                v0,
                v1,
            )
        )
    # ... and a shelf between them traps the keeper tongue above the plug.
    adds.append(
        prism_uw(
            [
                (wall_inner_u(C3_GOAL_BOTTOM_W) - 0.4, C3_GOAL_BOTTOM_W),
                (C3_STOP_U1, C3_GOAL_BOTTOM_W),
                (C3_STOP_U1, C3_GOAL_TOP_W),
                (wall_inner_u(C3_GOAL_TOP_W) - 0.4, C3_GOAL_TOP_W),
            ],
            C3_VC - 11.8,
            C3_VC + 11.8,
        )
    )
    adds.append(
        box3(
            C3_BOSS_U - KEEPER_BLOCK_HALF,
            C3_BOSS_U + KEEPER_BLOCK_HALF,
            C3_VC - 7.0,
            C3_VC + 7.0,
            base,
            BOARD_TOP_W,
        )
    )
    w0, w1 = FLOOR + 0.2, 12.4
    cuts.append(
        prism_uw(
            [
                (-3.0, w0),
                (wall_inner_u(w0) + 0.4, w0),
                (wall_inner_u(w1) + 0.4, w1),
                (-3.0, w1),
            ],
            C3_VC - 7.5,
            C3_VC + 7.5,
        )
    )

    # ---- RD-03D slot in the back wall ------------------------------------
    half = RADAR_WID / 2.0
    slot_half = half + RADAR_SLOT_FIT
    back_half = half - RADAR_RAIL_GRIP + RADAR_SLOT_FIT
    slot_bottom = RADAR_BOTTOM_Y - 0.5
    open_top = 40.0  # past the sloping rim; the envelope trims what is added
    floor_y = -(RADAR_REF_W - FLOOR - 0.3) / COS_D
    for side in (-1.0, 1.0):
        x0, x1 = sorted((side * back_half, side * (half + 5.0)))
        adds.append(
            # Rails run down to the floor so they have no underside to droop.
            W_FRAME.place(box3(x0, x1, floor_y, open_top, WALL - 0.2, WALL + RADAR_RAIL_PROUD))
        )
    # The window is open to the top of the wall.  The radar SoC stands about
    # 1 mm proud of the antenna face and has to pass down through here.
    cuts.append(
        W_FRAME.place(
            box3(
                -(half - RADAR_EDGE_GRIP),
                half - RADAR_EDGE_GRIP,
                RADAR_BOTTOM_Y + RADAR_EDGE_GRIP,
                open_top,
                -2.0,
                RADAR_LIP_T + 0.2,
            )
        )
    )
    cuts.append(
        W_FRAME.place(
            box3(-slot_half, slot_half, slot_bottom, open_top, RADAR_LIP_T, RADAR_LIP_T + RADAR_SLOT_T)
        )
    )
    cuts.append(
        W_FRAME.place(
            box3(-back_half, back_half, slot_bottom, open_top, RADAR_LIP_T + RADAR_SLOT_T - 0.1, WALL + 4.0)
        )
    )
    # Relief below the board for the 1.25 mm plug and its leads.
    cuts.append(
        W_FRAME.place(
            box3(-5.0, 5.0, max(slot_bottom - 14.0, floor_y), slot_bottom + 0.1, RADAR_LIP_T + 0.8, WALL + 4.0)
        )
    )

    # ---- power bay: charger on two rails, fenced, no screws --------------
    for side in (-1.0, 1.0):
        rail_u = CHARGER_UC + side * CHARGER_HOLE_DY
        adds.append(
            box3(
                rail_u - 2.5,
                rail_u + 2.5,
                CHARGER_VC - CHARGER_LEN / 2.0,
                CHARGER_VC + CHARGER_LEN / 2.0,
                base,
                CHARGER_PCB_BOTTOM_W,
            )
        )
    # Fence on the distal side; it is also one side of the regulator pocket.
    adds.append(
        box3(
            CHARGER_U0 - CHARGER_FIT - FENCE_T,
            CHARGER_U0 - CHARGER_FIT,
            -V_IN - 0.5,
            FENCE_V1,
            base,
            FENCE_TOP_W,
        )
    )
    # Stop at the JST end, in the proximal corner, clear of both plugs.
    adds.append(
        box3(
            CHARGER_U1 - 6.0,
            CHARGER_U1 + CHARGER_FIT + 0.5,
            CHARGER_VC + CHARGER_LEN / 2.0 + CHARGER_FIT,
            DIVIDER_V0 + 0.5,
            base,
            FENCE_TOP_W,
        )
    )
    # Open-topped notch so the USB-C and barrel jack drop straight in.
    cuts.append(
        box3(
            CHARGER_UC - 11.2,
            CHARGER_UC + 13.3,
            -V_OUT - 1.0,
            -V_IN + 0.3,
            CHARGER_PCB_BOTTOM_W - 0.5,
            RIM_H + 2.0,
        )
    )

    # ---- power bay: battery pocket and regulator pocket ------------------
    rib_top = FLOOR + 8.0
    adds.append(box3(BLOCK_D_U1 - 0.5, BATTERY_U1 - 6.0, BATTERY_V1, BATTERY_V1 + 3.0, base, rib_top))
    # End wall: the pouch cannot slide onto its own lead, which leaves past
    # the wall's free end.  The far side of the wall is the regulator pocket.
    adds.append(box3(BATTERY_STOP_U, REG_POCKET_U0, -V_IN - 0.5, REG_V1_FACE + 3.5, base, rib_top))
    # Short return that closes part of the pocket's fourth side; the rest
    # stays open for the regulator's wires.
    adds.append(
        box3(BATTERY_STOP_U, REG_POCKET_U0 + 3.0, REG_V1_FACE, REG_V1_FACE + 3.5, base, FLOOR + 5.0)
    )

    # ---- power bay: cover bolt blocks ------------------------------------
    for u0, u1, v0, v1 in COVER_BLOCKS:
        adds.append(box3(u0, u1, v0, v1, base, RIM_H))

    # ---- bolt holes --------------------------------------------------------
    for site in side_nut_sites():
        cuts.append(site.cut)
    for u, v, tilted in TRAY_SCREWS:
        screw_frame = tray_screw_frame(u, v, tilted)
        cuts.append(screw_frame.place(vcyl(M3_CLEARANCE_D, 0.0, 0.0, -FLOOR - 2.0, 1.0)))
        if tilted:
            cuts.append(screw_frame.place(vcyl(TRAY_SCREW_SPOTFACE_D, 0.0, 0.0, 0.0, 6.0)))

    solid = body.fuse(*adds).intersect(outer).cut(*cuts)
    return cleaned(solid)


# --------------------------------------------------------------------------
# Bezel, cover, keepers
# --------------------------------------------------------------------------
@lru_cache(maxsize=None)
def make_bezel() -> cq.Shape:
    """Flat display frame, returned in the tray frame."""
    lift = BEZEL_T / COS_A
    plate = prism_yz(
        [
            (SLOPE_V, RIM_H),
            (V_OUT, APEX_W),
            (V_OUT, APEX_W + lift),
            (SLOPE_V, RIM_H + lift),
        ],
        -40.0,
        TRAY_L,
    )
    # Trim the distal end flush with the leaning distal wall.
    top = APEX_W + 40.0
    plate = plate.intersect(
        prism_uw(
            [(0.0, 0.0), (wall_outer_u(top), top), (TRAY_L, top), (TRAY_L, 0.0)],
            -V_OUT - 1.0,
            V_OUT + 1.0,
        )
    )

    cuts = []
    half_l = DISPLAY_LCD_LENGTH / 2.0 + BEZEL_WINDOW_CLEARANCE
    half_w = DISPLAY_LCD_WIDTH / 2.0 + BEZEL_WINDOW_CLEARANCE
    cuts.append(D_FRAME.place(box3(-half_l, half_l, -half_w, half_w, -2.0, BEZEL_T + 4.0)))
    # Relief over the header solder stubs and the LCD flex at the header end.
    cuts.append(
        D_FRAME.place(
            box3(
                -(DISPLAY_PCB_LENGTH / 2.0 + 1.2),
                -half_l + 0.1,
                -(DISPLAY_HEADER_HALF_SPAN + 1.5),
                DISPLAY_HEADER_HALF_SPAN + 1.5,
                -1.0,
                2.0,
            )
        )
    )
    radius = M3_CLEARANCE_D / 2.0
    for hp, hq in DISPLAY_HOLES:
        cuts.append(D_FRAME.place(vcyl(M3_CLEARANCE_D, hp, hq, -2.0, BEZEL_T + 4.0)))
        # The display's holes sit 1 mm from the glass, too close for a web
        # worth printing, so each bolt hole opens into the window.
        p0, p1 = sorted((hp, (half_l - 0.1) * (1.0 if hp > 0 else -1.0)))
        cuts.append(D_FRAME.place(box3(p0, p1, hq - radius, hq + radius, -2.0, BEZEL_T + 4.0)))
    return cleaned(plate.cut(*cuts))


@lru_cache(maxsize=None)
def make_power_cover() -> cq.Shape:
    plate = box3(
        wall_outer_u(COVER_TOP_W), TRAY_L, -V_OUT, SLOPE_V - COVER_GAP, RIM_H, COVER_TOP_W
    )
    charger_top = CHARGER_PCB_TOP_W
    pillar_v0 = CHARGER_VC + 7.5
    pillar_v1 = CHARGER_VC + CHARGER_LEN / 2.0 - 0.05
    adds = [
        # Broad pad that leaves BATTERY_FOAM_GAP for a foam strip over the pouch.
        box3(
            BATTERY_UC - 20.0,
            BATTERY_UC + 20.0,
            BATTERY_VC - 13.0,
            BATTERY_VC + 13.0,
            FLOOR + BATTERY_T + BATTERY_FOAM_GAP,
            RIM_H + 0.3,
        ),
        # Pillar that keeps the regulator in its pocket.
        box3(REG_UC - 4.0, REG_UC + 4.0, REG_VC - 4.5, REG_VC + 4.5, REG_TOP_W + 0.4, RIM_H + 0.3),
        # Two pillars hold the JST end of the charger down on its rails.  The
        # distal one overhangs the fence, which stops below it.
        box3(CHARGER_UC - 18.7, CHARGER_UC - 11.2, pillar_v0, pillar_v1, charger_top + 0.6, RIM_H + 0.3),
        box3(CHARGER_UC + 11.2, CHARGER_UC + 16.9, pillar_v0, pillar_v1, charger_top + 0.6, RIM_H + 0.3),
        # Low pad over the barrel jack holds the other end down.
        box3(
            CHARGER_UC - 10.0,
            CHARGER_UC - 3.0,
            -V_IN + 0.8,
            CHARGER_VC - 9.0,
            charger_top + 11.0 + 0.3,
            RIM_H + 0.3,
        ),
    ]
    cuts = [
        vcyl(M3_CLEARANCE_D, boss_u, boss_v, RIM_H - 1.0, COVER_TOP_W + 1.0)
        for boss_u, boss_v, _ in COVER_BOSSES
    ]
    # Peek hole over the charger's two status LEDs, which sit between the JSTs.
    cuts.append(vcyl(5.0, CHARGER_UC, CHARGER_VC + CHARGER_LED_X, RIM_H - 1.0, COVER_TOP_W + 1.0))
    return cleaned(plate.fuse(*adds).cut(*cuts))


@lru_cache(maxsize=None)
def make_s3_keeper() -> cq.Shape:
    v0, v1 = S3_VC - S3_KEEPER_W / 2.0, S3_VC + S3_KEEPER_W / 2.0
    bar = box3(S3_BOSS_U - KEEPER_BLOCK_HALF, S3_U_USB_END, v0, v1, BOARD_TOP_W, KEEPER_TOP_W)
    # Tongue for the wall mortise, stepped up so it stays above the USB-C
    # plug overmoulds.
    tongue = box3(
        S3_U_USB_END - 0.1,
        U_IN_END + S3_MORTISE_DEPTH - 0.5,
        v0,
        v1,
        BOARD_TOP_W + S3_TONGUE_STEP,
        KEEPER_TOP_W,
    )
    hole = vcyl(M3_CLEARANCE_D, S3_BOSS_U, S3_VC, BOARD_TOP_W - 1.0, KEEPER_TOP_W + 1.0)
    return cleaned(bar.fuse(tongue).cut(hole))


@lru_cache(maxsize=None)
def make_c3_keeper() -> cq.Shape:
    v0, v1 = C3_VC - C3_KEEPER_W / 2.0, C3_VC + C3_KEEPER_W / 2.0
    bar = box3(C3_U_USB_END - 0.1, C3_BOSS_U + KEEPER_BLOCK_HALF, v0, v1, BOARD_TOP_W, KEEPER_TOP_W)
    # Tongue that hooks under the shelf on the distal wall; stepped up so it
    # stays above a USB-C plug overmould.
    tongue = box3(C3_TONGUE_U0, C3_U_USB_END, v0, v1, BOARD_TOP_W + C3_TONGUE_STEP, KEEPER_TOP_W)
    hole = vcyl(M3_CLEARANCE_D, C3_BOSS_U, C3_VC, BOARD_TOP_W - 1.0, KEEPER_TOP_W + 1.0)
    return cleaned(bar.fuse(tongue).cut(hole))


@lru_cache(maxsize=None)
def make_test_coupon() -> cq.Shape:
    """Half-hour test print of the three fits everything else depends on.

    Left to right: a cover-bolt block with its side nut slot, the saddle's
    drop-in nut pocket above a bolt hole that starts on the bed, and a short
    length of the radar slot.  Drawn in build orientation.
    """
    base = box3(0.0, 72.0, 0.0, 18.0, 0.0, 3.0)
    adds = [
        box3(2.0, 15.5, 0.0, 11.5, 0.0, 15.0),
        box3(20.0, 34.0, 2.0, 16.0, 0.0, 12.0),
        box3(36.0, 70.0, 0.0, WALL + RADAR_RAIL_PROUD, 0.0, 13.0),
    ]
    site = NutSite(
        "coupon",
        (Frame(origin=(8.75, 7.0, 15.0), x_dir=(0.0, 1.0, 0.0), z_dir=(0.0, 0.0, 1.0)),),
        COVER_NUT_COVER,
        COVER_NUT_COVER + NUT_SLOT_H + 1.0,
        6.5,
    )
    radius = M3_CLEARANCE_D / 2.0
    half = RADAR_WID / 2.0
    back_half = half - RADAR_RAIL_GRIP + RADAR_SLOT_FIT
    xc = 53.0
    cuts = [
        site.cut,
        vcyl(M3_CLEARANCE_D, 27.0, 9.0, -1.0, SADDLE_NUT_SEAT + 0.5),
        hex_prism(NUT_POCKET_AF, SADDLE_NUT_SEAT, 20.0).translate(Vec(27.0, 9.0, 0.0)),
        cq.Solid.makeCone(radius + 0.9, radius, 0.9, Vec(27.0, 9.0, -0.1), Vec(0.0, 0.0, 1.0)),
        box3(xc - half + RADAR_EDGE_GRIP, xc + half - RADAR_EDGE_GRIP, -1.0, RADAR_LIP_T + 0.2, 3.0, 20.0),
        box3(xc - half - RADAR_SLOT_FIT, xc + half + RADAR_SLOT_FIT, RADAR_LIP_T, RADAR_LIP_T + RADAR_SLOT_T, 3.0, 20.0),
        box3(xc - back_half, xc + back_half, RADAR_LIP_T + RADAR_SLOT_T - 0.1, 20.0, 3.0, 20.0),
    ]
    return cleaned(base.fuse(*adds).cut(*cuts))


# --------------------------------------------------------------------------
# Purchased components, fasteners, wiring aids (all in the tray frame)
# --------------------------------------------------------------------------
@lru_cache(maxsize=None)
def load_component(slug: str) -> cq.Shape:
    path = COMPONENT_MODELS / f"{slug}.step"
    if not path.is_file():
        raise FileNotFoundError(f"Generate the component models first: missing {path}")
    return cq.Compound.makeCompound(cq.importers.importStep(str(path)).vals())


@dataclass(frozen=True)
class Item:
    name: str
    shape: cq.Shape
    color: tuple[float, float, float]
    note: str = ""
    source: str = ""


def component_items() -> list[Item]:
    display = D_FRAME.place(
        load_component("n203_msp3520_display_approx").translate(Vec(0, 0, PCB_BACK_R))
    )
    s3 = S3_FRAME.place(load_component("n203_yd_esp32_s3_approx"))
    c3 = C3_FRAME.place(load_component("n203_esp32_c3_supermini_approx"))
    radar = W_FRAME.place(RADAR_LOCAL_FRAME.place(load_component("n203_rd03d_v1_approx")))
    charger = CHARGER_FRAME.place(load_component("n203_adafruit_bq24074_4755_approx"))
    battery = load_component("n203_jljlup_lipo_2000mah_approx").translate(
        Vec(BATTERY_UC, BATTERY_VC, FLOOR)
    )
    regulator = load_component("n203_pololu_s13v20f5_item4085_approx").translate(
        Vec(REG_UC, REG_VC, REG_Z_OFFSET)
    )
    return [
        Item(
            "display_msp3520",
            display,
            (0.07, 0.16, 0.12),
            "Landscape along the forearm, facing the thumb side; header toward the elbow; clamped between bezel pads and four tray corbels by four bolts and nuts",
            "n203_msp3520_display_approx",
        ),
        Item(
            "esp32_s3",
            s3,
            (0.08, 0.30, 0.70),
            "Component side down, header pins up; both USB-C ports at the elbow-end wall",
            "n203_yd_esp32_s3_approx",
        ),
        Item(
            "esp32_c3",
            c3,
            (0.16, 0.45, 0.88),
            "Component side down, header pins up; USB-C at the distal wall",
            "n203_esp32_c3_supermini_approx",
        ),
        Item(
            "rd03d_radar",
            radar,
            (0.10, 0.55, 0.25),
            "In the back wall behind the screen, antenna face outward (+Y), long axis upright, connector end down",
            "n203_rd03d_v1_approx",
        ),
        Item(
            "bq24074_charger",
            charger,
            (0.10, 0.42, 0.75),
            "On two rails inside a fence, held down by the cover; USB-C and barrel jack through the -Y wall notch",
            "n203_adafruit_bq24074_4755_approx",
        ),
        Item(
            "lipo_battery",
            battery,
            (0.74, 0.74, 0.70),
            "Ribbed pocket under the power cover; foam gap, no hard clamp",
            "n203_jljlup_lipo_2000mah_approx",
        ),
        Item(
            "pololu_regulator",
            regulator,
            (0.25, 0.68, 0.30),
            "Pocket between the battery stop wall and the charger fence, held down by a cover pillar",
            "n203_pololu_s13v20f5_item4085_approx",
        ),
    ]


def _screw(diameter: float, length: float, head_d: float, head_h: float) -> cq.Shape:
    """Pan-head screw: head sits on z = 0, shank runs toward -z."""
    head = cq.Solid.makeCylinder(head_d / 2.0, head_h, Vec(0, 0, 0), Vec(0, 0, 1))
    shank = cq.Solid.makeCylinder(diameter / 2.0, length, Vec(0, 0, -length), Vec(0, 0, 1))
    return head.fuse(shank)


@dataclass(frozen=True)
class Fastener:
    name: str
    spec: str
    shape: cq.Shape
    joins: str


def fastener_items() -> list[Fastener]:
    items: list[Fastener] = []
    m3x12 = _screw(3.0, 12.0, M3_HEAD_D, M3_HEAD_H)
    m3x14 = _screw(3.0, 14.0, M3_HEAD_D, M3_HEAD_H)
    nut = "M3 hex nut"
    for index, (u, v, tilted) in enumerate(TRAY_SCREWS, start=1):
        frame = tray_screw_frame(u, v, tilted)
        seat = -tray_screw_deck_drop(tilted) - SADDLE_NUT_SEAT
        items.append(Fastener(f"tray_bolt_{index}", "M3x12 pan head", frame.place(m3x12), "tray to saddle deck"))
        items.append(
            Fastener(
                f"tray_nut_{index}",
                nut,
                frame.place(hex_prism(M3_NUT_AF, seat - M3_NUT_T, seat)),
                "tray to saddle deck (pocket on the arm side)",
            )
        )
    sites = {site.name: site for site in side_nut_sites()}
    for index, (hp, hq) in enumerate(DISPLAY_HOLES, start=1):
        items.append(
            Fastener(
                f"display_bolt_{index}",
                "M3x14 pan head",
                D_FRAME.place(m3x14.translate(Vec(hp, hq, BEZEL_T))),
                "bezel + display to tray corbel",
            )
        )
        items.append(Fastener(f"display_nut_{index}", nut, sites[f"display_nut_{index}"].nut, "bezel + display to tray corbel"))
    for index, (boss_u, boss_v, _) in enumerate(COVER_BOSSES, start=1):
        items.append(
            Fastener(
                f"cover_bolt_{index}",
                "M3x12 pan head",
                m3x12.translate(Vec(boss_u, boss_v, COVER_TOP_W)),
                "power cover to tray block",
            )
        )
        items.append(Fastener(f"cover_nut_{index}", nut, sites[f"cover_nut_{index}"].nut, "power cover to tray block"))
    for board, boss_u, boss_v in (("s3", S3_BOSS_U, S3_VC), ("c3", C3_BOSS_U, C3_VC)):
        items.append(
            Fastener(
                f"{board}_keeper_bolt",
                "M3x12 pan head",
                m3x12.translate(Vec(boss_u, boss_v, KEEPER_TOP_W)),
                "keeper bar to tray block",
            )
        )
        items.append(Fastener(f"{board}_keeper_nut", nut, sites[f"{board}_keeper_nut"].nut, "keeper bar to tray block"))
    return items


def connector_keepouts() -> list[Item]:
    """Space that plugged-in jumper housings and cable plugs need."""
    color = (0.95, 0.80, 0.15)
    items: list[Item] = []
    r1 = PCB_BACK_R - HEADER_PLASTIC_H
    r0 = r1 - DUPONT_HOUSING_L
    items.append(
        Item(
            "keepout_display_header_jumpers",
            D_FRAME.place(
                box3(
                    DISPLAY_HEADER_P - 1.3,
                    DISPLAY_HEADER_P + 1.3,
                    -DISPLAY_HEADER_HALF_SPAN,
                    DISPLAY_HEADER_HALF_SPAN,
                    r0,
                    r1,
                )
            ),
            color,
            "14 jumper housings on the rear display header",
        )
    )
    w0 = BOARD_TOP_W + HEADER_PLASTIC_H
    w1 = w0 + DUPONT_HOUSING_L
    s3_half = 11.0 * 2.54
    for side, label in ((-1.0, "minus_y"), (1.0, "plus_y")):
        row_v = S3_VC + side * S3_ROW_SPACING / 2.0
        items.append(
            Item(
                f"keepout_s3_{label}_row_jumpers",
                box3(S3_UC - s3_half, S3_UC + s3_half, row_v - 1.3, row_v + 1.3, w0, w1),
                color,
                "jumper housings on the full S3 header row",
            )
        )
    c3_half = 4.0 * 2.54
    for side, label in ((-1.0, "minus_y"), (1.0, "plus_y")):
        row_v = C3_VC + side * C3_ROW_SPACING / 2.0
        items.append(
            Item(
                f"keepout_c3_{label}_row_jumpers",
                box3(C3_UC - c3_half, C3_UC + c3_half, row_v - 1.3, row_v + 1.3, w0, w1),
                color,
                "jumper housings on the full C3 header row",
            )
        )
    items.append(
        Item(
            "keepout_radar_plug",
            W_FRAME.place(
                box3(-3.5, 3.5, RADAR_BOTTOM_Y - 9.0, RADAR_BOTTOM_Y, RADAR_PCB_BACK_Z, RADAR_PCB_BACK_Z + 4.0)
            ),
            color,
            "1.25 mm radar plug and lead exit",
        )
    )
    jst_face_v = CHARGER_JST_FACE_V
    for dy, label in ((-5.72, "battery"), (5.72, "load")):
        plug_u = CHARGER_UC - dy
        items.append(
            Item(
                f"keepout_charger_{label}_jst_plug",
                box3(plug_u - 3.2, plug_u + 3.2, jst_face_v, jst_face_v + 9.0, CHARGER_PCB_TOP_W, CHARGER_PCB_TOP_W + 5.0),
                color,
                "JST-PH plug and lead exit",
            )
        )
    return items


def usb_plug_keepouts() -> list[Item]:
    """USB-C plug overmoulds approaching from outside (12.6 x 6.8 mm, 25 long)."""
    color = (0.85, 0.30, 0.25)
    items: list[Item] = []
    usb_center_w = BOARD_BOTTOM_W - USB_C_SHELL_H / 2.0
    for index, offset in enumerate(S3_USB_PORT_OFFSETS, start=1):
        face_u = S3_U_USB_END + 1.20
        items.append(
            Item(
                f"keepout_s3_usb_plug_{index}",
                box3(
                    face_u + 0.5,
                    face_u + 25.5,
                    S3_VC + offset - 6.3,
                    S3_VC + offset + 6.3,
                    usb_center_w - 3.4,
                    usb_center_w + 3.4,
                ),
                color,
            )
        )
    face_u = C3_U_USB_END - 2.00
    items.append(
        Item(
            "keepout_c3_usb_plug",
            box3(face_u - 25.5, face_u - 0.5, C3_VC - 6.3, C3_VC + 6.3, usb_center_w - 3.4, usb_center_w + 3.4),
            color,
        )
    )
    charger_usb_w = CHARGER_PCB_TOP_W + USB_C_SHELL_H / 2.0
    face_v = CHARGER_VC - 20.07
    usb_u = CHARGER_UC + 6.35
    items.append(
        Item(
            "keepout_charger_usb_plug",
            box3(usb_u - 6.3, usb_u + 6.3, face_v - 25.5, face_v - 0.5, charger_usb_w - 3.4, charger_usb_w + 3.4),
            color,
        )
    )
    return items


def _tube(points: list[tuple[float, float, float]], diameter: float) -> cq.Shape:
    solids: list[cq.Shape] = []
    radius = diameter / 2.0
    for start, end in zip(points, points[1:]):
        a, b = Vec(*start), Vec(*end)
        delta = b - a
        if delta.Length < 1e-6:
            continue
        solids.append(cq.Solid.makeCylinder(radius, delta.Length, a, delta.normalized()))
    for point in points:
        solids.append(cq.Solid.makeSphere(radius, Vec(*point)))
    return cq.Compound.makeCompound(solids)


@dataclass(frozen=True)
class WireRoute:
    name: str
    description: str
    points: tuple[tuple[float, float, float], ...]
    diameter: float
    color: tuple[float, float, float]

    @property
    def shape(self) -> cq.Shape:
        return _tube(list(self.points), self.diameter)

    @property
    def length(self) -> float:
        return sum(
            (Vec(*b) - Vec(*a)).Length for a, b in zip(self.points, self.points[1:])
        )


def wire_routes() -> list[WireRoute]:
    """Reference harness paths through free space, in the tray frame.

    With the boards as mounted here, ESP32-S3 header J1 (GPIO4..GPIO14, 5V,
    GND) is the +Y row, next to the back wall, and J3 (GPIO41/GPIO42) is the
    -Y row.  On the ESP32-C3 the 5V/GND/3V3/GPIO4 row is the -Y row and the
    GPIO5..GPIO21 row is the +Y row.
    """
    jumper_top = BOARD_TOP_W + HEADER_PLASTIC_H + DUPONT_HOUSING_L
    over = jumper_top + 2.0
    high = jumper_top + 5.0
    tip_r = PCB_BACK_R - HEADER_PLASTIC_H - DUPONT_HOUSING_L
    header_mid = D_FRAME.point(DISPLAY_HEADER_P, 0.0, tip_r - 3.0)
    header_low = D_FRAME.point(DISPLAY_HEADER_P, DISPLAY_HEADER_HALF_SPAN - 2.5, tip_r - 2.0)
    s3_j1_v = S3_VC + S3_ROW_SPACING / 2.0
    s3_j3_v = S3_VC - S3_ROW_SPACING / 2.0
    c3_minus = C3_VC - C3_ROW_SPACING / 2.0
    c3_plus = C3_VC + C3_ROW_SPACING / 2.0
    radar_plug = W_FRAME.point(0.0, RADAR_BOTTOM_Y - 6.0, RADAR_PCB_BACK_Z + 2.5)

    lane_v = (CHARGER_JST_FACE_V + 0.56 + DIVIDER_V0) / 2.0  # between charger and divider
    lane_w = FLOOR + 8.0
    trunk_u = JST_NOTCH_U[0] + 3.0
    splice = (trunk_u, LANE_V, FLOOR + 8.0)
    floor_w = FLOOR + 3.0
    reg_tap = (REG_UC, REG_VC + 9.5, FLOOR + 6.0)
    jst_w = CHARGER_PCB_TOP_W + 2.5
    prox_u = U_IN_END - 13.5  # just short of the proximal corbels
    s3_power_u = S3_U_USB_END - 7.6  # 5V pin, second from the USB end
    return [
        WireRoute(
            "display_bus",
            "Display header (9 signals) to ESP32-S3 header J1, GPIO6..GPIO14",
            (
                header_mid,
                (prox_u, 38.0, over + 1.5),
                (S3_UC + 8.0, s3_j1_v, over),
            ),
            4.5,
            (0.90, 0.45, 0.10),
        ),
        WireRoute(
            "scanner_uart",
            "ESP32-C3 GPIO21 (TX) / GPIO20 (RX) / GND to ESP32-S3 GPIO41 / GPIO42 / GND",
            (
                (C3_U_FAR_END - 2.0, c3_plus, over),
                (C3_U_FAR_END - 2.0, c3_plus, high),
                (C3_BOSS_U + 6.0, 25.0, high),
                (S3_U_FAR_END + 17.8, 25.0, high),
                (S3_U_FAR_END + 17.8, s3_j3_v, over),
            ),
            3.5,
            (0.20, 0.55, 0.90),
        ),
        WireRoute(
            "radar_lead",
            "RD-03D TX to ESP32-C3 GPIO4 (5 V and GND join the splice)",
            (
                radar_plug,
                (RADAR_U - 16.0, 40.0, high),
                (C3_UC - 1.3, 37.0, high),
                (C3_UC - 1.3, c3_minus, over),
            ),
            3.5,
            (0.15, 0.70, 0.35),
        ),
        WireRoute(
            "charger_load_to_regulator",
            "Charger LOAD socket along the lane to regulator VIN + GND",
            (
                (CHARGER_UC - 5.72, CHARGER_JST_FACE_V + 7.0, jst_w),
                (CHARGER_U0 - 3.0, lane_v, lane_w),
                (REG_UC, lane_v, lane_w),
                reg_tap,
            ),
            3.0,
            (0.60, 0.10, 0.60),
        ),
        WireRoute(
            "power_5v_trunk",
            "Regulator 5 V + GND along the lane and through the divider notch to the splice",
            (
                reg_tap,
                (REG_UC + 3.5, lane_v, lane_w + 3.5),
                (trunk_u, lane_v, lane_w + 3.5),
                (trunk_u, 3.0, lane_w + 3.5),
                splice,
            ),
            3.0,
            (0.85, 0.10, 0.10),
        ),
        WireRoute(
            "power_5v_to_s3",
            "Splice to the ESP32-S3 5V and GND pins (USB end of header J1)",
            (
                splice,
                (prox_u, LANE_V, FLOOR + 8.0),
                (prox_u, 16.0, over),
                (s3_power_u, 16.0, over),
                (s3_power_u, s3_j1_v, over),
            ),
            3.0,
            (0.85, 0.10, 0.10),
        ),
        WireRoute(
            "power_5v_to_display",
            "Splice to display VCC and GND, the two lowest header pins",
            (
                splice,
                (prox_u, LANE_V, FLOOR + 8.0),
                (prox_u, 16.0, over),
                header_low,
            ),
            3.0,
            (0.85, 0.10, 0.10),
        ),
        WireRoute(
            "power_5v_to_c3_and_radar",
            "Splice along the wiring lane to the ESP32-C3 5V/GND pins and the radar lead",
            (
                splice,
                (trunk_u - 8.0, LANE_V, floor_w),
                (C3_U_USB_END + 4.2, LANE_V, floor_w),
                (C3_U_USB_END + 4.2, 9.5, over),
                (C3_U_USB_END + 4.2, c3_minus, over),
            ),
            3.0,
            (0.85, 0.10, 0.10),
        ),
        WireRoute(
            "battery_lead",
            "LiPo JST-PH lead across the gap and along the lane to the charger BATT socket",
            (
                (BATTERY_STOP_U + 5.0, BATTERY_VC + 12.0, FLOOR + 8.0),
                (BATTERY_STOP_U + 5.0, lane_v, lane_w - 3.5),
                (CHARGER_UC + 5.72, lane_v, lane_w - 3.5),
                (CHARGER_UC + 5.72, CHARGER_JST_FACE_V + 7.0, jst_w),
            ),
            3.0,
            (0.60, 0.10, 0.60),
        ),
    ]


# --------------------------------------------------------------------------
# Assembly helpers
# --------------------------------------------------------------------------
PRINTED_COLORS = {
    "saddle": (0.40, 0.43, 0.48),
    "tray": (0.66, 0.69, 0.74),
    "bezel": (0.22, 0.23, 0.27),
    "power_cover": (0.52, 0.55, 0.60),
    "s3_keeper": (0.93, 0.55, 0.12),
    "c3_keeper": (0.93, 0.55, 0.12),
}


def printed_parts_tray_frame() -> dict[str, cq.Shape]:
    """Printed parts that live in the tray frame (everything but the saddle)."""
    return {
        "tray": make_tray(),
        "bezel": make_bezel(),
        "power_cover": make_power_cover(),
        "s3_keeper": make_s3_keeper(),
        "c3_keeper": make_c3_keeper(),
    }


def to_bracer(shape: cq.Shape) -> cq.Shape:
    return tray_frame().place(shape)


def print_oriented_parts() -> dict[str, cq.Shape]:
    """Every printed part turned to its intended build orientation, on z = 0."""
    flip = (Vec(0, 0, 0), Vec(1, 0, 0), 180.0)

    def seat(shape: cq.Shape) -> cq.Shape:
        bb = shape.BoundingBox()
        return shape.translate(Vec(-bb.xmin, -(bb.ymin + bb.ymax) / 2.0, -bb.zmin))

    saddle = tray_frame().unplace(make_saddle()).rotate(*flip)
    bezel = D_FRAME.unplace(make_bezel()).rotate(*flip)
    cover = make_power_cover().rotate(*flip)
    return {
        "saddle": seat(saddle),
        "tray": seat(make_tray()),
        "bezel": seat(bezel),
        "power_cover": seat(cover),
        "s3_keeper": seat(make_s3_keeper().rotate(*flip)),
        "c3_keeper": seat(make_c3_keeper().rotate(*flip)),
        "test_coupon": seat(make_test_coupon()),
    }

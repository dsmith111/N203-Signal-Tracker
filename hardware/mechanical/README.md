# N203 component CAD

These models are mechanical references for enclosure design. They are either
based on published board outlines or are conservative approximations. They must not be
used to design a PCB footprint. Measure the physical parts before a final print,
especially connector positions, solder joints, wire exits, and component height.

## Generated models

Run `generate_component_models.py` to create the files under `models/`. They
are generated output and are not kept in git:

| Model | Nominal source dimensions |
|---|---|
| `n203_msp3520_display_approx.step` | 98.00 x 56.34 mm PCB, 92.00 x 49.50 mm mounting pattern, 5.80 mm front stack, 14.18 mm total including header |
| `n203_yd_esp32_s3_approx.step` | 63.50 x 27.94 mm clone PCB; component and dual-USB geometry is approximate |
| `n203_esp32_c3_supermini_approx.step` | 22.00 x 18.00 mm PCB, approximately 24.00 mm including USB-C |
| `n203_rd03d_v1_approx.step` | 44.00 x 15.00 mm PCB; antenna face on +Z, 1x4 connector on the back at the -X end; heights approximate |
| `n203_pololu_s13v20f5_item4085_approx.step` | Envelope with the 8.89 x 12.07 mm outline and 5.37 mm height of Pololu's published model; internal detail not modelled |
| `n203_gy521_mpu6050_approx.step` | Common GY-521 layout, 21.00 x 16.00 x 3.80 mm; supplied board and header must be checked |
| `n203_jljlup_lipo_2000mah_approx.step` | Seller-listed 52.00 x 34.00 x 10.00 mm protected pouch; lead and PH2.0 plug omitted |
| `n203_dfrobot_c4001_sen0609_approx.step` | Official 26.00 x 30.00 mm board outline, 22.00 mm hole spacing and 2.00 mm holes; Z geometry approximate |
| `n203_adafruit_bq24074_4755_approx.step` | Official 38.10 x 33.02 mm board outline, 33.02 x 27.94 mm mounting pattern, and connector X/Y from the Eagle board; heights approximate |

Each detailed model also has an STL preview and a `_keepout.step` file. A
keepout is a simple rectangular cutter expanded by 0.50 mm on every XY side,
0.50 mm below the component, and 0.80 mm above it. Importing a keepout into
Fusion and using it as a Combine/Cut tool gives a deliberately loose first
prototype cavity.

The LiPo keepout is deliberately different: it adds 1.50 mm on each XY side,
1.00 mm below, and 2.00 mm above, resulting in a nominal 55 x 37 x 13 mm
cavity. Do not clamp, puncture, sharply bend, or place screw tips against the
pouch. Use a broad, removable retainer and leave a separate route for the
approximately 70 mm lead and PH2.0 connector.

The display keepout includes the full rear header depth. If the display will be
hard-wired without the protruding header, make a shallower custom pocket from
the detailed model instead.

## Coordinate convention

- X and Y are centered on the PCB.
- Z = 0 is the back surface of the PCB.
- Positive Z points toward the component/front side.
- The display header extends into negative Z. Its row is 2.00 mm inside the
  -X short edge; the active area is offset 2.33 mm toward +X.
- USB and sensor connector ends point toward negative X.
- The RD-03D antenna face is +Z. Its connector is on the back (negative Z) at
  the -X end, behind the 2x2 patch group, and mates from that end.
- The C4001 pin edge points toward negative Y and its radar face toward positive Z.
- The BQ24074 USB-C and barrel-jack edge points toward negative X.

## Regenerating

From the repository root in PowerShell:

```powershell
python -m venv build\cadquery-venv
.\build\cadquery-venv\Scripts\python.exe -m pip install -r hardware\mechanical\requirements-cad.txt
.\build\cadquery-venv\Scripts\python.exe hardware\mechanical\generate_component_models.py
```

The generator writes `models/manifest.json` with the actual bounding box of
every exported model. All editable dimensions are grouped near the top of the
Python file.

## Source drawings

- LCDWiki MSP3520 mechanical drawing:
  <https://www.lcdwiki.com/res/MSP3520/MSP3520_Size.pdf>
- Matching YD-ESP32-S3 clone description:
  <https://github.com/profharris/YD-ESP32-S3_ESP32-S3-WROOM-1_Dev>
- ESP32-C3 SuperMini physical measurements:
  <https://github.com/sigmdel/supermini_esp32c3_sketches>
- Ai-Thinker RD-03D specification:
  <https://aithinker-static.oss-cn-shenzhen.aliyuncs.com/docs/_media_old/rd-03d_v1.0.0_specification20240103.pdf>
- Pololu S13V20F5 item 4085 resources and manufacturer STEP:
  <https://www.pololu.com/product/4085/resources>
- GY-521 (MPU-6050) module: seller-listed dimensions, cross-checked against a
  common community CAD envelope:
  <https://marathon-os.com/library/gy521-mpu6050-accelerometer-and-68155822fcc7c6ee36fcf89e>
- JLJLUP protected 2000 mAh LiPo: seller-listed dimensions, 52 x 34 x 10 mm.
- DFRobot C4001 SEN0609 wiki and mechanical drawing:
  <https://wiki.dfrobot.com/sen0609>
- Adafruit BQ24074 item 4755 and open PCB source:
  <https://www.adafruit.com/product/4755>
  <https://github.com/adafruit/Adafruit-BQ24074-PCB>

Pololu's STEP file is not redistributed here. To use it for local CAD work,
see `vendor/pololu/README.md`.

The BQ24074 outline and mounting coordinates were derived from Adafruit's open
PCB source, designed by Limor Fried/Ladyada and published under CC BY-SA. The
connector bodies in this model are simplified clearance envelopes, not library
package replicas.

## Right-arm bracer

`bracer_v3/` is a printable forearm mount for the whole tracker; see
`bracer_v3/README.md` for print, assembly, and wiring instructions. Its
generator checks the design against the component models above, so run
`generate_component_models.py` first.

The brace is shaped from the forearm measurements in `arm_fit.py`. It ships
with a nominal reference arm; `bracer_v3/README.md` explains how to use your
own.

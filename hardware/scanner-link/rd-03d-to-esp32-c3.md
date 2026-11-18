# RD-03D to ESP32-C3 wiring and firmware

Status: electrical, protocol, raw framing, range, axis orientation, and tracked
position software gates complete; final production hardware soak pending.

For the complete transition from the temporary direct C4001 bench, use the
[`RD-03D next-session checklist`](rd-03d-next-session.md).

The Ai-Thinker RD-03D V1 specification defines a four-pin 1.25 mm connector:

```text
pin 1  5V   power input
pin 2  GND  ground
pin 3  TX   radar UART output
pin 4  RX   radar UART input
```

Follow the labels and pin-1 mark on the physical module. Cable colors are not
part of the specification and must not be used to infer the pin order.

## Electrical contract

- Supply: regulated 5 V, with at least 200 mA available to the radar.
- Supply ripple: below 100 mV.
- UART I/O: 3.3 V.
- UART: 256000 baud, 8 data bits, no parity, 1 stop bit.
- Initial capture is read-only; no configuration command is sent.

Power everything down before changing the wiring. Use one known 5 V source for
the radar. If that is an external supply, join its ground to C3 ground but do
not join two independently powered 5 V rails. The C3 board's 5V header should
only supply the radar if that USB/power path has been verified to retain the
required current and voltage at the module.

## Initial wiring

```text
RD-03D 5V  -> verified 5 V supply capable of >=200 mA
RD-03D GND -> supply ground and C3 GND
RD-03D TX  -> optional 47-100 ohm series resistor -> C3 GPIO4
RD-03D RX  -> disconnected
```

The existing C3-to-S3 link remains:

```text
C3 GPIO21 TX -> S3 GPIO41 RX
S3 GPIO42 TX -> C3 GPIO20 RX
C3 GND       -> S3 GND
```

C3 GPIO6 is reserved for the later command path to RD-03D RX. Do not connect
it during raw capture. GPIO5 is not needed by this module; the alternate C4001
path uses it for `OUT`.

## Capture firmware

Build, upload, and monitor the RDP-friendly environment:

```powershell
pio run -e c3-rd03d-capture-rdp
pio run -e c3-rd03d-capture-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-rd03d-capture-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The firmware reads UART0 on GPIO4 at 256000 baud. It searches for the official
fixed 30-byte report:

```text
AA FF 03 00 + three 8-byte target slots + 55 CC
```

It prints the first 32 accepted or rejected candidates as 60 hexadecimal
characters. Type `c` in the monitor to capture another 32. Captures are bounded
and no bytes are retained indefinitely.

Expected healthy diagnostics:

```text
[link] connected=yes ... frames=0 ...
[radar] mode=rd03d-capture baud=256000 raw_rx_bytes=<increasing> frames=<increasing> rejected=0 ...
[rd03d.raw] frame=1 data=AAFF0300................................................55CC
```

`link frames=0` is intentional: this phase does not fabricate canonical radar
data. The S3 should retain the scanner descriptor/link connection but show its
radar data as stale/offline until the normalized driver replaces the capture
build.

Capture separate batches for an empty area, one stationary person, approach,
retreat, lateral crossing, and power-cycle recovery. Multi-target mode is not
commanded yet; the factory default is single-target mode.

The capture target intentionally produces no display contacts. After real
frames pass this gate, they become replay fixtures for the normalized RD-03D
driver that will publish X/Y contacts through the existing C3-to-S3 link.

## Staged live-range firmware

After the raw-capture gate is clean, the read-only
`c3-rd03d-range-live-rdp` target can drive the display before native X/Y
orientation has been verified:

```powershell
pio run -e c3-rd03d-range-live-rdp
pio run -e c3-rd03d-range-live-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-range-live-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Keep the same wiring, including RD-03D RX and C3 GPIO6 disconnected. The
decoder consumes the three official target slots, derives each range as
`sqrt(x*x + y*y)`, converts speed from cm/s to mm/s, and publishes up to three
range-only contacts through the normal scanner link. The display therefore
places contacts on its unknown-bearing centerline; overlapping ranges may
overlap visually. It does not yet expose native X/Y, azimuth, or stable track
identity.

Expected healthy diagnostics include:

```text
[link] connected=yes ... frames=<increasing> ...
[rd03d.range] contacts=1 first_range_mm=<nonzero> first_velocity_mm_s=<signed> ...
[radar] mode=rd03d-range-live baud=256000 raw_rx_bytes=<increasing> frames=<increasing> rejected=0 valid_range=<increasing> ...
```

If raw framing is not already proven on the physical unit, return to the
capture target rather than diagnosing interpreted range data.

## Position-orientation verification

After range-live succeeds, flash the position-verification build:

```powershell
pio run -e c3-rd03d-position-verify-rdp
pio run -e c3-rd03d-position-verify-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-position-verify-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

No wiring changes are required. Keep RD-03D RX and C3 GPIO6 disconnected. Fix
the module in its intended orientation and use one whole-person target at a
time. The available 1.4-1.5 m depth is sufficient for this gate. Maintain a
small amount of in-place body motion at each mark; physical captures showed
that native position was substantially more stable than when the target stood
perfectly motionless. Type `p` at each mark to capture eight concise
`[rd03d.xy]` samples:

1. Centerline: `abs(x_mm)` should be small relative to positive `y_mm`, and the
   dot should be centered.
2. Radar-right: viewed from behind the radar looking into its sensing area,
   canonical `x_mm` and azimuth should be positive and the dot should move
   right. `native_x_mm` should be negative.
3. Radar-left: canonical X and azimuth should be negative and the dot should
   move left. Native X should be positive.
4. Approach: range and Y should decrease; velocity should be negative.
5. Retreat: range and Y should increase; velocity should be positive.

Physical testing verified the sign relationship above, so native X is inverted
exactly once when the canonical contact is built. Each `[rd03d.xy]` line keeps
both `native_x_mm` and canonical `x_mm` visible. The position build is quiet by
default. Its monitor commands are `p` for eight
X/Y frames, `q` to cancel a batch, `v` to toggle all link/system/UART/IMU
diagnostics, and `h` for help. Retain each short `[rd03d.xy]` batch. If
left/right ever appears mirrored after a mounting change, diagnose the physical
orientation rather than adding another display-side inversion.

The target publishes position but does not advertise track identity. RD-03D
slots are snapshot-local until a software association test proves otherwise.

## Production tracked-position firmware

After the orientation test passes, use the normal live position target with the
same read-only wiring:

```powershell
pio run -e c3-rd03d-position-live-rdp
pio run -e c3-rd03d-position-live-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-position-live-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

This target publishes canonical X/Y, range, azimuth, radial velocity, confidence,
and stable software IDs. Association is bounded to three contacts and does not
trust the RD-03D slot number as identity. A small position deadband and adaptive
filter reduce jitter; a missing contact remains visible for 300 ms and its ID is
retained internally for 1.2 seconds for reacquisition. Type `v` to enable full
diagnostics. A concise `[rd03d.track]` line is emitted when the published contact
count changes.

The MPU-6050 is optional. Its presence or absence has no effect on RD-03D contact
creation, association, smoothing, or the frames sent to the display.

## Primary references

- [Ai-Thinker RD-03D resources](https://docs.ai-thinker.com/en/Rd-03D_V1/)
- [RD-03D Specification V1.0.0](https://en.ai-thinker.com/Uploads/file/20231016/20231016032622_13559.pdf)
- [RD-03D Multi-Target Trajectory Tracking User Manual](https://aithinker-static.oss-cn-shenzhen.aliyuncs.com/docs/_media_old/rd-03d_multi-target_trajectory_tracking_user_manual.pdf)
- [RD-03D Quick Start V1.0.0](https://aithinker-static.oss-cn-shenzhen.aliyuncs.com/docs/_media_old/rd-03dquickstartdocument.pdf)

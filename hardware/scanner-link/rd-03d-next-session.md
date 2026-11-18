# Next hardware session: ESP32-C3 + RD-03D

Status: raw framing, range, and physical X/Y orientation gates have passed. The
normal separated C3 scanner and S3 display path now uses the tracked-position
firmware below; capture and verification builds remain available for diagnosis.

The first RD-03D session is a raw-data characterization bench. It proves the
sensor UART and records trustworthy target frames before the normalizer is
enabled. The C3/S3 link can be connected and healthy during this bench, but the
capture firmware intentionally publishes no canonical radar frames, so real
X/Y dots are not expected on the display yet.

## Power-down transition

1. Power down the S3, C3, display, and sensor before moving any wire.
2. Remove every C4001 connection from the setup. Retain the C4001 firmware and
   documentation as an optional diagnostic fixture.
3. Do not connect a C4001 and RD-03D to C3 GPIO4 at the same time. GPIO4 is the
   receive pin for whichever sensor backend the selected C3 target owns.
4. Restore the separated C3/S3 UART and common ground:

```text
C3 GPIO21 TX -> S3 GPIO41 RX
S3 GPIO42 TX -> C3 GPIO20 RX
C3 GND       -> S3 GND
```

5. Wire the RD-03D read-only capture path:

```text
RD-03D 5V  -> verified regulated 5 V supply capable of >=200 mA
RD-03D GND -> supply ground and C3 GND
RD-03D TX  -> optional 47-100 ohm series resistor -> C3 GPIO4
RD-03D RX  -> disconnected
C3 GPIO6   -> disconnected
```

6. If the MPU-6050 is included, connect its 3.3 V, ground, SDA GPIO0, and SCL
   GPIO1 wiring from
   [`mpu6050-to-esp32-c3.md`](mpu6050-to-esp32-c3.md). The firmware remains
   operational if the IMU is absent.

Follow the module's pin-1 mark and labels, not cable colors. Do not join two
independently powered 5 V rails. The complete electrical contract is in
[`rd-03d-to-esp32-c3.md`](rd-03d-to-esp32-c3.md).

## Firmware to load

Restore the normal scanner-driven display target on the S3:

```powershell
pio run -e s3-scanner-display-rdp
pio run -e s3-scanner-display-rdp -t upload --upload-port S3_COM_NUMBER
```

Load the read-only RD-03D capture target on the C3:

```powershell
pio run -e c3-rd03d-capture-rdp
pio run -e c3-rd03d-capture-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-capture-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The expected healthy C3 evidence is:

```text
[link] connected=yes ... frames=0 ...
[radar] mode=rd03d-capture baud=256000 raw_rx_bytes=<increasing> frames=<increasing> rejected=0 ...
[rd03d.raw] frame=1 data=AAFF0300................................................55CC
[imu] state=ready address=0x68 hz=<near 100> ...
```

`link frames=0` and a stale/offline radar presentation on the S3 are expected
in this target. They mean the capture build is refusing to invent normalized
positions before its input has been verified; they do not mean the inter-board
link has failed.

Once the raw-capture evidence is clean, load the staged range-only target to
exercise the complete RD-03D-to-display path without claiming that native X/Y
orientation has been verified:

```powershell
pio run -e c3-rd03d-range-live-rdp
pio run -e c3-rd03d-range-live-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-range-live-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

It publishes up to three derived-range contacts on the display's
unknown-bearing centerline. Leave RD-03D RX and C3 GPIO6 disconnected.

After that passes, use the position-verification target with the same wiring:

```powershell
pio run -e c3-rd03d-position-verify-rdp
pio run -e c3-rd03d-position-verify-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-position-verify-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Use the center/right/left/approach/retreat matrix in
[`rd-03d-to-esp32-c3.md`](rd-03d-to-esp32-c3.md) and retain the position logs.

For normal display operation, promote the C3 to the tracked-position build:

```powershell
pio run -e c3-rd03d-position-live-rdp
pio run -e c3-rd03d-position-live-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-position-live-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The MPU-6050 may remain disconnected. It is optional telemetry and is not an
input to the production RD-03D tracker.

## Capture set

The target prints the first 32 candidates after boot. Type `c` in the C3 USB
monitor for another 32-frame batch. Keep separate captures for:

- empty room;
- one centered stationary person;
- approach and retreat;
- left-to-right and right-to-left lateral crossing;
- entry and exit;
- RD-03D power-cycle and C3 reconnect;
- at least five minutes idle.

During the same session, retain IMU diagnostics for stationary-table, walking,
slow/rapid rotation, shaking, and hinge-deployment motion. Keep the complete
scanner carrier still during the two-second startup bias collection.

For each capture, retain the complete `[rd03d.raw]` lines plus the surrounding
`[radar]` counters. A clean accepted report is exactly 30 bytes, begins
`AAFF0300`, and ends `55CC`. Repeated corruption or a rising rejection count is
a wiring, power, or signal-integrity problem to resolve before decoder work.

## Exit gate

The software normalization, axis transform, and stable-tracking layers are now
implemented. Final hardware acceptance requires:

- `connected=yes` with increasing link and RD-03D frame counters;
- stable center/right/left and approach/retreat display behavior;
- selection and lock following the same person during ordinary movement;
- disappearance and re-entry behavior matching the documented 300 ms visible
  hold and 1.2-second identity retention window;
- a two-person crossing check if the module emits multiple targets;
- at least 30 minutes without growing parser rejection, reset, or heap-failure
  counters;
- automatic recovery when the C3 and S3 are power-cycled independently.

The MPU may remain disconnected for every item in this gate.

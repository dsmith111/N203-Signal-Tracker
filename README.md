# N203 Signal Tracker

N203 Signal Tracker is a portable radar-display project. The desktop emulator,
ESP32-C3 scanner firmware, and ESP32-S3 display firmware consume the same
canonical radar model.

## Phase 1 quick start

The native emulator has no third-party runtime dependencies.  It renders the
logical 480 x 320 display either in a live window on Windows or to a portable
PPM image for deterministic regression testing.

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug
.\build\Debug\n203-display-emulator.exe scenarios\display\tracking-three.json output.ppm
```

Launch the real-time emulator with:

```powershell
.\build\Debug\n203-display-emulator.exe scenarios\display\tracking-three.json --live
```

The window refreshes at approximately 30 FPS. Click contacts to select and lock
them, click the range buttons to change scale, and press Escape to exit.

Use `--frames` to render a particular animation frame and `--tap x y` to inject
a touch at logical display coordinates:

```powershell
.\build\Debug\n203-display-emulator.exe scenarios\display\tracking-single.json single.ppm --frames 90 --tap 448 270
```

`output.ppm` can be opened by most image viewers, converted by ImageMagick, or
used directly by a screenshot comparison tool.

## ILI9488 hardware bench

The first ESP32-S3 target validates power, backlight sequencing, the ILI9488
SPI link, landscape orientation, color order, and PSRAM before the shared N203
renderer is introduced. Follow the wiring and meter checks in
[`hardware/display/ili9488-xpt2046.md`](hardware/display/ili9488-xpt2046.md),
then build, upload, and monitor it with:

```powershell
pio run -e s3-display-bench
pio run -e s3-display-bench -t upload
pio device monitor -e s3-display-bench
```

The bench cycles solid colors and a geometry page while printing a once-per-second
heartbeat. See [`experiments/display-bench/README.md`](experiments/display-bench/README.md)
for the expected observations.

After the panel bench passes, the shared hardware demo runs the same
`DisplayApp` and `RadarRenderer` used by the desktop emulator:

```powershell
pio run -e s3-display-demo-rdp
pio run -e s3-display-demo-rdp -t upload --upload-port COM_NUMBER
```

See [`experiments/display-demo/README.md`](experiments/display-demo/README.md)
for native-USB/RDP upload and monitoring instructions.

Once display performance is stable, validate and calibrate the XPT2046 without
letting touch events affect the tracker UI:

```powershell
pio run -e s3-touch-bench-rdp
pio run -e s3-touch-bench-rdp -t upload --upload-port COM_NUMBER
```

See [`experiments/touch-bench/README.md`](experiments/touch-bench/README.md) for
the raw-touch checks and guided calibration procedure.

## Real scanner link

The production display target replaces canned frames with typed, COBS-framed,
CRC-protected canonical radar frames received over a dedicated UART:

```powershell
pio run -e s3-scanner-display-rdp
pio run -e s3-scanner-display-rdp -t upload --upload-port COM_NUMBER
```

The S3 uses GPIO41 for scanner RX and GPIO42 for the `HelloAck` return path
while native USB remains dedicated to upload and diagnostics. See
[`docs/radar-link-v2.md`](docs/radar-link-v2.md) for the byte contract and
[`hardware/scanner-link/esp32-c3-to-s3-uart.md`](hardware/scanner-link/esp32-c3-to-s3-uart.md)
for the electrical handoff.

The simulated scanner-node target for the supplied 16-pin ESP32-C3 board is:

```powershell
pio run -e c3-scanner-sim-rdp
pio run -e c3-scanner-sim-rdp -t upload --upload-port COM_NUMBER
```

Its accepted pin assignment and staged wiring are in
[`hardware/scanner-link/esp32-c3-supermini.md`](hardware/scanner-link/esp32-c3-supermini.md).

For RD-03D diagnostics, the capture target reads its documented
256000-baud UART without changing module settings or publishing interpreted
contacts:

```powershell
pio run -e c3-rd03d-capture-rdp
pio run -e c3-rd03d-capture-rdp -t upload --upload-port COM_NUMBER
```

Follow the 5 V, 3.3 V UART, and read-only wiring contract in
[`hardware/scanner-link/rd-03d-to-esp32-c3.md`](hardware/scanner-link/rd-03d-to-esp32-c3.md).
The next hardware session returns to the separated C3 + S3 architecture; its
exact power-down, rewiring, firmware, expected-output, and capture sequence is
in
[`hardware/scanner-link/rd-03d-next-session.md`](hardware/scanner-link/rd-03d-next-session.md).
This diagnostic target keeps the inter-board link alive but intentionally sends
no positions. The staged live-range target
publishes up to three derived-range contacts through the same link:

```powershell
pio run -e c3-rd03d-range-live-rdp
pio run -e c3-rd03d-range-live-rdp -t upload --upload-port COM_NUMBER
```

It renders range on the unknown-bearing centerline. The physical native-axis
gate has passed; this build remains available when directional data needs to be
excluded during diagnosis.

The dedicated verification target enables those directional fields while
logging the native coordinates for a controlled center/left/right test:

```powershell
pio run -e c3-rd03d-position-verify-rdp
pio run -e c3-rd03d-position-verify-rdp -t upload --upload-port COM_NUMBER
```

The verified production path adds software association, stable IDs, position
smoothing, and short dropout retention while continuing to use the same S3
display firmware:

```powershell
pio run -e c3-rd03d-position-live-rdp
pio run -e c3-rd03d-position-live-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-rd03d-position-live-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

This is the normal RD-03D target after raw framing and orientation have been
verified. It keeps RD-03D RX disconnected and requires no MPU-6050.

The current v1 software status, automated 30-minute pipeline soak, and remaining
hardware-only acceptance checks are recorded in
[`docs/v1-software-status.md`](docs/v1-software-status.md).

The C3 firmware can probe an optional MPU-6050 on GPIO0/GPIO1 and emit
100 Hz gyro/accelerometer telemetry without driving display warnings. See
[`hardware/scanner-link/mpu6050-to-esp32-c3.md`](hardware/scanner-link/mpu6050-to-esp32-c3.md)
for safe 3.3 V wiring and startup calibration.

The completed C4001 SEN0609 paths remain available as independent diagnostic
fixtures and do not replace the RD-03D implementation:

```powershell
pio run -e c3-c4001-capture-rdp
pio run -e c3-c4001-capture-rdp -t upload --upload-port COM_NUMBER
```

After the raw UART gate passes, its presence-only live target publishes strict
canonical frames to the existing S3 scanner display:

```powershell
pio run -e c3-c4001-live-rdp
pio run -e c3-c4001-live-rdp -t upload --upload-port COM_NUMBER
```

If the C4001 is producing `$DFDMD` distance/speed reports, use the separated
range-live target instead. It renders measured range on the unknown-bearing
centerline through the normal C3/S3 link:

```powershell
pio run -e c3-c4001-range-live-rdp
pio run -e c3-c4001-range-live-rdp -t upload --upload-port COM_NUMBER
```

See [`hardware/scanner-link/c4001-to-esp32-c3.md`](hardware/scanner-link/c4001-to-esp32-c3.md)
for its 3.3/5 V VIN, UART, optional `OUT` wiring, and both live modes.

For a temporary single-S3 presence-display test, a separate target connects
the C4001 TX directly to S3 GPIO41 while leaving the permanent C3/S3 design
unchanged:

```powershell
pio run -e s3-c4001-display-rdp
pio run -e s3-c4001-display-rdp -t upload --upload-port S3_COM_NUMBER
```

Disconnect the C3 UART wires first and follow
[`hardware/scanner-link/c4001-direct-s3.md`](hardware/scanner-link/c4001-direct-s3.md).

To configure the C4001 for single-target distance/speed reports and draw its
measured range on the display centerline, connect S3 GPIO42 to C4001 RX and use:

```powershell
pio run -e s3-c4001-position-display-rdp
pio run -e s3-c4001-position-display-rdp -t upload --upload-port S3_COM_NUMBER
```

This is range with unknown bearing, not true X/Y position. The C4001 mode is
saved; the monitor command `p` restores saved presence mode and `r` selects
range mode.

## Project layout

- `firmware/display-s3` - ESP32-S3 display-device configuration and adapters.
- `firmware/scanner-c3` - ESP32-C3 scanner controller and link adapter.
- `components/radar/protocol` - portable scanner-link encoder and stream decoder.
- `components/radar/c4001` - bounded C4001 framing, strict presence/range
  decoding, transition filtering, and canonical frame construction.
- `components/radar/rd03d` - bounded RD-03D framing plus staged range and
  velocity normalization.
- `components/radar/simulated` - deterministic scanner scenarios for link testing.
- `components/motion` - portable IMU samples, gyro-bias calibration, gravity
  filtering, and motion telemetry metrics.
- `experiments/display-bench` - isolated ILI9488 hardware bring-up target.
- `experiments/display-demo` - shared tracker UI running from compiled demo frames.
- `experiments/touch-bench` - isolated XPT2046 diagnostics and calibration.
- `hardware/display` - accepted display wiring and electrical constraints.
- `hardware/scanner-link` - accepted C3, S3, and radar-link wiring.

- `components/radar/core` — canonical sensor-independent radar contracts.
- `components/display` — state, interaction and renderer shared by every display target.
- `emulations/display` — native scenario runner and raster display adapter.
- `scenarios/display` — portable JSON examples for the desktop emulator.
- `tests/unit` — dependency-free unit tests for the portable components.

The intended architecture and the current display contract are documented in
[`docs/display-contract.md`](docs/display-contract.md).

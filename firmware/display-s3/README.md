# ESP32-S3 display firmware

This directory owns board-specific ILI9488, XPT2046, backlight, clock, and radar
link adapters. The `s3-display-demo` target composes them with compiled demo
frames; `s3-scanner-display` replaces that source with CRC-protected canonical
frames received on the dedicated scanner UART.

`LovyanGfxCanvas` maps every portable drawing primitive to LovyanGFX and holds
one SPI transaction across each complete render pass. The shared 3 x 5 bitmap
font keeps desktop and hardware text metrics identical.

`DirtyTileFramebuffer` owns two RGB565 PSRAM sprites and presents only changed
16 x 16 tiles to the SPI panel. Horizontally adjacent tiles are coalesced into
one transfer region.

`UartRadarSource` drains the scanner UART into the portable v2 COBS stream
decoder, acknowledges scanner `Hello` packets on the return UART, keeps only
the newest complete frame, and reports session, COBS, CRC, format, payload,
sequence gap, duplicate, restart, and superseded-frame counters over USB CDC.

## Scanner-display target

Build and upload through the confirmed native-USB/RDP path with:

```powershell
pio run -e s3-scanner-display-rdp
pio run -e s3-scanner-display-rdp -t upload --upload-port COM_NUMBER
```

After tapping RST and redirecting the application COM port, monitor with:

```powershell
pio device monitor -e s3-scanner-display-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

With no scanner sender connected, the screen remains in `WARMUP` and `[link]`
reports zero packets and frames. With a healthy 10 Hz sender, the S3 prints the
negotiated scanner descriptor, `connected=yes`, and an increasing frame count;
rejection, gap, duplicate, and reset counters remain zero. If valid traffic
stops after at least one frame, the display changes to `DATA HOLD` using the
S3's local receive clock.

The current wire specification is
[`docs/radar-link-v2.md`](../../docs/radar-link-v2.md).

## Temporary direct-C4001 target

`s3-c4001-display` is a bench-only path for connecting the C4001 UART directly
to S3 GPIO41 while keeping the intended C3/S3 architecture intact. It reuses
the strict C4001 decoder, two-report presence filter, canonical frame model,
display state machine, touch input, and renderer. It does not initialize a UART
TX pin or send commands to the sensor.

```powershell
pio run -e s3-c4001-display-rdp
pio run -e s3-c4001-display-rdp -t upload --upload-port S3_COM_NUMBER
pio device monitor -e s3-c4001-display-rdp --port S3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Use this target only with the C3 UART wires disconnected from S3 GPIO41 and
GPIO42. The temporary wiring and expected observations are in
[`hardware/scanner-link/c4001-direct-s3.md`](../../hardware/scanner-link/c4001-direct-s3.md).

### C4001 centered-range setup

`s3-c4001-position-display` adds S3 GPIO42 as the command TX path and switches
the sensor to its documented distance/speed mode. `$DFDMD` reports become one
range-only canonical contact with radial velocity and signal strength. Because
the C4001 supplies no bearing, the marker is deliberately drawn on the forward
centerline and is not advertised as an azimuth or X/Y measurement.

```powershell
pio run -e s3-c4001-position-display-rdp
pio run -e s3-c4001-position-display-rdp -t upload --upload-port S3_COM_NUMBER
pio device monitor -e s3-c4001-position-display-rdp --port S3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

This target requires `S3 GPIO42 -> C4001 RX`. Type `p` in the monitor to restore
and save presence mode before returning to a presence-only target; type `r` to
request range mode again. Exact wiring and the persistent-mode caveat are in
the same direct-C4001 hardware guide.

Authoritative wiring and electrical details are recorded in
[`hardware/display/ili9488-xpt2046.md`](../../hardware/display/ili9488-xpt2046.md).

## Touch calibration

`kTouchCalibration` in `include/board_config.hpp` was captured on one panel.
Resistive panels vary, so if taps land off-target, run
[`experiments/touch-bench`](../../experiments/touch-bench/README.md) on your
panel and replace the array with the line it prints.

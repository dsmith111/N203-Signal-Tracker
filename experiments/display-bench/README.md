# ILI9488 display bench

This target brings up the confirmed Hosyond/LCDWiki MSP3520-style ILI9488 SPI
module on the ESP32-S3-N16R8 board. It deliberately excludes the shared N203 UI
and XPT2046 reads so display wiring, color order, orientation, SPI stability,
backlight sequencing, and PSRAM configuration can be validated independently.

## Wiring prerequisite

Follow [`hardware/display/ili9488-xpt2046.md`](../../hardware/display/ili9488-xpt2046.md).
Before attaching module VCC, use a meter to verify the selected ESP32-S3 header
marked `5V` measures approximately 5 V relative to GND while powered through the
USB connector that will be used. Leave J1 open.

## Build and upload

```powershell
pio run -e s3-display-bench
pio run -e s3-display-bench -t upload
pio device monitor -e s3-display-bench
```

This clone's connector labeled `USB` is the confirmed working native USB
power/data interface. Its connector labeled `COM` does not enumerate and is not
used. The firmware enables USB CDC on boot for heartbeat output.

For a native USB port redirected through RDP, first put the board into download
mode locally (hold BOOT, tap RST, release BOOT), then reconnect RDP after the
download-mode COM port has enumerated. Upload without automatic DTR/RTS resets:

```powershell
pio run -e s3-display-bench-rdp -t upload --upload-port COM_NUMBER
```

After upload completes, tap RST locally. The application USB COM port may need
to be redirected again before monitoring:

```powershell
pio device monitor -e s3-display-bench --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The target starts at a conservative 20 MHz LCD write clock and cycles through
solid red, green, blue, white, black, and a 480 x 320 geometry page. Serial
output reports fill/draw time, free heap, PSRAM detection, and a heartbeat.

Expected first-pass observations:

- No backlight flash before initialization.
- Correct red/green/blue order.
- Geometry page is landscape at exactly 480 x 320.
- `psram_found=yes` and `psram_size=8388608` in the serial log.
- No corrupted pixels or spontaneous resets during a 30-minute loop.

Touch CS is held HIGH. LCD SDO and all SD-card pads remain disconnected.

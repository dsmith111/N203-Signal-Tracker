# Shared N203 display demo

This target runs the real portable `DisplayApp` and `RadarRenderer` on the
ESP32-S3/ILI9488 through `LovyanGfxCanvas`. It uses compiled radar frames rather
than JSON, a filesystem, or live radar hardware. The calibrated XPT2046 sends
release-edge input through the same `DisplayApp` interface as desktop clicks.

The demo cycles every 4.5 seconds through warmup, empty tracking, one locked
contact, three contacts with a selected track, range-only, presence-only, stale
data, and scanner-offline states. Serial statistics report frame rate, average
composition and transfer time, worst total frame time, dirty-pixel coverage,
free heap, and free PSRAM.

Rendering uses two RGB565 framebuffers in PSRAM. The portable renderer composes
the next frame off-screen, compares it with the previous frame in 16 x 16
tiles, and coalesces adjacent changed tiles before sending them over SPI. This
keeps static grid, bezel, text, and scanline pixels off the bus while preserving
the exact desktop-renderer output.

## Build and upload through RDP

Put the board into download mode before reconnecting RDP: hold BOOT, tap RST,
then release BOOT. After the native USB COM port is redirected, run:

```powershell
pio run -e s3-display-demo-rdp
pio run -e s3-display-demo-rdp -t upload --upload-port COM_NUMBER
```

Tap RST after upload. Reconnect RDP if the application COM port is assigned or
redirected again, then monitor with:

```powershell
pio device monitor -e s3-display-demo-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

## Touch interaction

The target applies the calibration captured for the assembled panel and filters
the resistive controller at 100 Hz. A tap must last at least 20 ms, release is
debounced across two polls, and the emitted coordinate is the median of the last
five samples.

During `tracking-one` and `tracking-three`, tap a contact once to select it and
again to toggle target lock. Tap the `-` and `+` controls at the lower left to
change display range. Each accepted release prints an `[input]` line containing
its mapped coordinate and resulting UI state. The range controls retain their
inset artwork but their touch regions extend to the physical bottom edge.

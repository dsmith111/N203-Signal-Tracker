# XPT2046 touch bench

This isolated target validates the XPT2046 wiring and captures calibration
values before touch input is allowed to affect the tracker UI.

## Build, upload, and monitor through RDP

Put the board into download mode locally (hold BOOT, tap RST, release BOOT),
redirect its native USB COM port into the RDP session, then run:

```powershell
pio run -e s3-touch-bench-rdp
pio run -e s3-touch-bench-rdp -t upload --upload-port COM_NUMBER
```

Tap RST after upload, redirect the application COM port again if necessary,
and open the monitor:

```powershell
pio device monitor -e s3-touch-bench-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Touch the panel in several places first. The screen and serial output should
show changing raw X/Y values, nonzero pressure, `IRQ:LOW` while pressed, and a
single `up` event after release.

Type `c` in the serial monitor to begin calibration. Touch the tip of each of
the four corner markers in sequence and lift the stylus or finger fully between
markers. When complete, the firmware prints an eight-value calibration array.
Tap all four corners and the center once more; the mapped preview should follow
the contact and report coordinates near `(0,0)`, `(479,0)`, `(0,319)`,
`(479,319)`, and `(240,160)` respectively.

Calibration is intentionally not persisted by this diagnostic target. Save the
printed `[calibration] cpp=...` line for integration into the tracker firmware.

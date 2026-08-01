# Display emulation

The Phase 1 native adapter uses an in-memory 480 x 320 raster canvas. On Windows
it can present that canvas continuously in a scaled live window; it can also
write a deterministic PPM screenshot on every platform. It deliberately
contains the scenario loader, window, screenshot writer and mouse-as-touch
adapter; none of them are dependencies of `components/display`.

The Win32 host is intentionally a desktop adapter. A future SDL/LovyanGFX host
can implement the same `DisplayCanvas` boundary without changing `DisplayApp` or
`RadarRenderer`.

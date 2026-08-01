# N203 Signal Tracker

N203 Signal Tracker is a portable radar-display project.  The first executable is
a desktop display emulator; it consumes the same canonical radar model that the
future ESP32 firmware will use.

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

## Project layout

- `components/radar/core` — canonical sensor-independent radar contracts.
- `components/display` — state, interaction and renderer shared by every display target.
- `emulations/display` — native scenario runner and raster display adapter.
- `scenarios/display` — portable JSON examples for the desktop emulator.
- `tests/unit` — dependency-free unit tests for the portable components.

The intended architecture and the current display contract are documented in
[`docs/display-contract.md`](docs/display-contract.md).

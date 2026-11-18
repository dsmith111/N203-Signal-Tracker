# Display contract

`components/display` is the portable display subsystem. It only consumes the
canonical `n203::radar::RadarFrame`, logical `DisplayInput` events and a clock;
it has no knowledge of scenario files, windows, board pins, UART or sensor
drivers.

## Logical coordinate system

The display has a fixed logical resolution of **480 x 320**. The motion-tracker
sector originates at `(240, 276)`, has a radius of `260` logical pixels and a
forward horizontal field of view of 80 degrees. Radar coordinates use
millimetres: positive `x` is right and positive `y` is forward/up on the
display. Contacts outside the selected display range or forward field of view
are not drawn.

## Radar model requirements

The display selects presentation from `RadarFrame::measurement_state` and
health. It only renders position when `Contact::valid_fields` has
`kContactPosition`. For a range-and-bearing contact it uses `kContactRange` and
`kContactAzimuth`; it makes no
claim that all radar sources provide either form.

Frames must pass `validateFrame()` before `DisplayApp` accepts them. The caller
supplies the local arrival time; a frame link silent for more than 1,200 ms is
shown as stale. The remote sensor timestamp is retained for diagnostics but is
never compared with the display board's independent clock. Offline, stale and
invalid-frame states have explicit warning presentation.

Presence is explicit: `unknown`, `clear`, and `detected` are not inferred from
whether a contact currently has a range or position. Carrier motion is also a
separate contract. `DopplerWarning` and `Stabilizing` add a visible warning
without hiding valid returns or disabling selection, locking, and range input.

## Input contract

A released touch in either range button changes the display scale among 2 m,
5 m, 10 m, 15 m and 20 m. Tapping a contact selects it; tapping that selected
contact again toggles target lock. These are presentation-level interactions and
do not command a radar source.

## Presentation contract

The normal view uses a constrained phosphor-green palette, a forward sector,
range bands, soft motion returns, sparse edge telemetry and restrained CRT
imperfections. It does not use a rotating 360-degree sweep.

The visual treatment must not reduce the underlying N203 capabilities. Contact
identity remains available for stable tracking; selecting a return reveals its
track number, and selecting it again toggles target lock. The compact `-` and
`+` controls remain visible so range zoom is discoverable rather than hidden
behind an undocumented gesture.

## Rendering boundary

`RadarRenderer` renders through `DisplayCanvas`, a minimal portable primitive
API. The native emulator implements it with an in-memory raster canvas; the S3
adapter can wrap a LovyanGFX device behind the same interface. This preserves
one layout, geometry and display-state implementation across both targets.

# Temporary direct C4001-to-ESP32-S3 bench

Status: bench-only presence and centered-range paths physically verified. The
intended system remains a separate C3 scanner controller and S3 display
controller; this path is retained as an optional diagnostic fixture.

## Isolate the normal scanner link

Power everything down before moving wires. Disconnect both C3/S3 UART signal
wires for this test:

```text
Disconnect C3 GPIO21 TX from S3 GPIO41 RX
Disconnect S3 GPIO42 TX from C3 GPIO20 RX
```

Do not leave the C3 TX and C4001 TX connected to S3 GPIO41 at the same time.
Two outputs driving one input wire can cause contention and corrupt the UART
stream. The C3 is not needed or powered for this direct test.

## Read-only presence wiring

Use the same verified, stable C4001 power arrangement that passed the C3
capture bench:

```text
C4001 TX   -> optional 47-100 ohm series resistor -> S3 GPIO41
C4001 RX   -> disconnected
C4001 OUT  -> disconnected
C4001 GND  -> S3 GND
C4001 VIN  -> verified stable 3.3 V supply
```

The S3 display wiring remains unchanged. If a separate regulated 3.3 V supply
is used for the sensor, join its ground to S3 ground and do not join separately
powered positive rails. The C4001 also accepts 5 V VIN, but no S3 GPIO may be
exposed to a 5 V signal; the already-verified 3.3 V bench arrangement is the
preferred temporary setup.

`OUT` is intentionally unused. The `s3-c4001-display-rdp` target consumes UART
reports, never configures an S3 UART TX pin, and sends no sensor commands.

## Build, upload, and monitor

```powershell
pio run -e s3-c4001-display-rdp
pio run -e s3-c4001-display-rdp -t upload --upload-port S3_COM_NUMBER
pio device monitor -e s3-c4001-display-rdp --port S3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The sensor UART is fixed at the verified 9600 baud. It is not probed at 115200.
The USB diagnostic monitor remains at 115200 baud.

After two matching valid reports, the display changes from `WARMUP` to the
presence-only screen and shows either `AREA CLEAR` or `PRES DETECT`. No contact
dots are expected because `$DFHPD` provides no target position or range.

Expected diagnostics:

```text
[sensor] waiting for strict C4001 $DFHPD/$DFDMD reports on GPIO41
[runtime] state=presence-only ...
[sensor] mode=c4001-direct report_mode=presence config=disabled connected=yes ...
```

Useful failure distinctions:

```text
rx=0
    No UART bytes reach GPIO41. Recheck power, common ground, and TX-to-GPIO41.

rx increasing, reports=0
    Bytes arrive but no complete '$' through '*' report is framed.

reports increasing, presence_valid=0 and range_valid=0
    Complete reports arrive but do not exactly match either supported report
    family. Check malformed, unsupported, and out_of_range counters.

presence_valid or range_valid increasing, connected=yes
    The direct sensor path is healthy. The display receives canonical frames.
```

If valid reports stop, `connected` changes to `no` and the existing display
arrival-time policy moves through stale/offline presentation instead of
inventing a clear state.

## Centered-range position setup

The C4001 is not a two-dimensional position sensor. In distance/speed mode it
reports one target's range, radial velocity, and energy, but no azimuth. N203
therefore draws the target at its measured range on the forward centerline.
This is an explicit unknown-bearing representation, not an inferred X/Y point.

Power down and add the return UART wire:

```text
C4001 TX -> S3 GPIO41
C4001 RX -> S3 GPIO42
C4001 OUT -> disconnected
C4001 GND -> S3 GND
C4001 VIN -> verified stable 3.3 V supply
```

Then build, upload, and monitor the dedicated setup target:

```powershell
pio run -e s3-c4001-position-display-rdp
pio run -e s3-c4001-position-display-rdp -t upload --upload-port S3_COM_NUMBER
pio device monitor -e s3-c4001-position-display-rdp --port S3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

If the sensor starts in presence mode, the target sends the official sequence
`sensorStop`, `setRunApp 1`, `saveConfig`, and `sensorStart`, then requires a
valid `$DFDMD` report before declaring configuration complete. The saved sensor
mode survives power cycles. If the sensor is already in range mode, no command
is sent.

Expected successful diagnostics:

```text
[sensor] position setup active; type p for presence mode or r for range mode
[runtime] state=range-only ...
[sensor] mode=c4001-direct report_mode=range config=complete connected=yes ... range_valid=... contacts=1 range_mm=1817 velocity_mm_s=129 ...
```

The dot moves only toward or away from the scanner along the centerline. Use
the display `+` control to select 15 m, 20 m, or 25 m when the contact is beyond
the current display range. A zero-target report produces `contacts=0`.

The configuration state progresses through `stopping`, `setting-mode`,
`saving`, and `verifying`. `config=failed` with `report_mode=presence` usually
means the S3 GPIO42-to-C4001 RX wire is absent, crossed, or unreliable.

To restore the sensor's saved presence mode, type `p` in the monitor and wait
for both of these fields:

```text
report_mode=presence config=complete
```

Type `r` to switch and save range mode again.

## Return to the separated architecture

If the next sensor path expects presence reports, restore presence mode with
`p` before disconnecting the position-setup target. Then flash
`s3-scanner-display-rdp`, remove both C4001 UART wires from the S3, and restore
the two C3/S3 UART wires. The direct targets do not modify C3 firmware, the
scanner-link protocol, or the normal S3 target.

For the planned C3 + RD-03D session, follow the exact power-down, rewiring,
firmware, and capture sequence in
[`rd-03d-next-session.md`](rd-03d-next-session.md). The RD-03D does not require
the C4001 to remain connected.

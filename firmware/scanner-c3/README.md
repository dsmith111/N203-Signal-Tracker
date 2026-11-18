# ESP32-C3 scanner firmware

The `c3-scanner-sim` target is the first physical scanner-node milestone. It
runs deterministic 10 Hz canonical radar scenarios, sends N203 link v2
`Hello` and `RadarFrame` packets to the S3, receives `HelloAck`, scans the
reserved I2C bus incrementally, and keeps native USB CDC available for
diagnostics.

The target uses the supplied 16-pin ESP32-C3 layout and a conservative 4 MB
ESP32-C3 DevKitM build profile. No radar module or IMU should be powered or
connected during this simulated-link bench.

```powershell
pio run -e c3-scanner-sim-rdp
pio run -e c3-scanner-sim-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-scanner-sim-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Because the RDP environment does not toggle reset control lines, enter the ROM
bootloader manually if upload cannot connect: hold BOOT, tap RESET, release
BOOT, then start the upload. After a successful upload, tap RESET and redirect
the re-enumerated application COM port before opening the monitor.

The USB monitor should show a new session identifier, the complete pin map,
the incremental I2C scan result, scenario changes, 10 transmitted frames per
second, and `connected=yes` after the S3 return wire carries a matching
`HelloAck`.

The inter-board link uses the Arduino core's canonical `Serial1` object routed
to GPIO20/GPIO21. UART0 is reserved for the later radar probe. The periodic
`[uart]` diagnostic reports the active controller, configured baud, GPIO20
input level, falling-edge count, and whether the pin-matrix assignment
succeeded. With GPIO21-to-GPIO20 loopback, both `edges` and link-level
`rx_bytes` must increase.

UART0 is initialized first and moved to GPIO4/GPIO6 before UART1 is attached to
GPIO20/GPIO21. This order matters on ESP32-C3 because GPIO20/GPIO21 are UART0's
ROM defaults. UART1 RX uses a one-byte FIFO threshold and one-symbol timeout so
short acknowledgment packets are delivered immediately.

For an isolated receive test, power down and disconnect both S3 signal wires,
then connect C3 GPIO21 directly to C3 GPIO20 and reset the C3. The link-level
`rx_bytes`, `rx_packets`, and `unexpected` counters must increase. The separate
`[radar] raw_rx_bytes` counter remains zero because it belongs to GPIO4.

See [`hardware/scanner-link/esp32-c3-supermini.md`](../../hardware/scanner-link/esp32-c3-supermini.md)
for the accepted board profile and bench wiring.

## MPU-6050 telemetry

Every C3 target now includes an optional, non-blocking MPU-6050 motion provider
on GPIO0 SDA and GPIO1 SCL. It probes both `0x68` and `0x69`, samples at about
100 Hz, estimates gyro bias from 200 startup samples, separates low-pass
gravity from dynamic acceleration, and prints raw axes plus derived magnitudes
in the existing USB diagnostics.

The IMU is telemetry-only at this phase: it does not set `DOPPLER WARNING` or
otherwise modify canonical frames. Missing or unplugged hardware is reprobed
without stopping radar UART input or the C3/S3 link. Follow
[`hardware/scanner-link/mpu6050-to-esp32-c3.md`](../../hardware/scanner-link/mpu6050-to-esp32-c3.md)
for the 3.3 V electrical gate, wiring, calibration posture, and capture set.

## RD-03D raw-capture target

`c3-rd03d-capture-rdp` is the retained framing-diagnostic target. It preserves the proven v2
Hello/HelloAck link but deliberately stops publishing simulated `RadarFrame`
packets. UART0 moves to the RD-03D's documented 256000-baud stream on GPIO4 and
a framing-only collector prints exact 30-byte reports to USB.

```powershell
pio run -e c3-rd03d-capture-rdp
pio run -e c3-rd03d-capture-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-rd03d-capture-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The first 32 valid or malformed candidates are printed. Type `c` in the monitor
to arm another 32-frame batch. The radar's RX pin stays disconnected: this
target makes no persistent or volatile configuration changes. A healthy module
increments `raw_rx_bytes` and `frames`, keeps `rejected=0`, and emits lines that
start with `AAFF0300` and end with `55CC`.

See [`hardware/scanner-link/rd-03d-to-esp32-c3.md`](../../hardware/scanner-link/rd-03d-to-esp32-c3.md)
for the verified power, logic-level, connector, and wiring contract.
Use the
[`next-session checklist`](../../hardware/scanner-link/rd-03d-next-session.md)
when changing back from the direct C4001/S3 bench. The raw-capture target does
not render positions; clean saved captures are the input to the following
RD-03D normalization phase.

## RD-03D staged live-range target

`c3-rd03d-range-live-rdp` turns clean RD-03D reports into canonical live
frames for the existing S3 display. It decodes all three documented target
slots and publishes presence, derived range, and radial velocity:

```powershell
pio run -e c3-rd03d-range-live-rdp
pio run -e c3-rd03d-range-live-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-range-live-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

This remains a read-only sensor path: RD-03D RX and C3 GPIO6 stay
disconnected. The display presents each target at its measured range on the
unknown-bearing centerline. Native X/Y and azimuth are intentionally withheld
until captures from known left/right positions verify the physical coordinate
orientation; slot numbers are not yet advertised as stable track identities.
Run `c3-rd03d-capture-rdp` first on new hardware and proceed only when accepted
frames rise while `rejected=0` remains stable.

## RD-03D position-verification target

`c3-rd03d-position-verify-rdp` is the next gate after range-live. It publishes
the RD-03D's native X/Y, derived range and azimuth, and radial velocity through
the canonical link so the existing positional UI renders real contacts:

```powershell
pio run -e c3-rd03d-position-verify-rdp
pio run -e c3-rd03d-position-verify-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-position-verify-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Physical testing with the observer facing the same direction as the radar
verified that native X has the opposite sign from the canonical display axis.
The scanner therefore inverts native X once at this boundary; canonical
positive X is display-right and positive Y remains forward. Diagnostics are
quiet by default.
After placing the target, type `p` for eight `[rd03d.xy]` samples; `q` cancels
the batch and `v` toggles the full link/system/UART/IMU diagnostics. Approaching
motion should report negative velocity.

This target deliberately omits `kTrackIdentity`: contact IDs are source slot
numbers and may be reassigned. Selection is suitable for this physical check,
but persistent locking waits for the software association layer.

## RD-03D production position target

`c3-rd03d-position-live-rdp` is the normal separated-scanner target after the
orientation gate passes. It converts native X to canonical display X, associates
the module's snapshot-local slots with stable software tracks, smooths small
position noise, and publishes up to three positional contacts:

```powershell
pio run -e c3-rd03d-position-live-rdp
pio run -e c3-rd03d-position-live-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-position-live-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Tracks remain visible for 300 ms across an isolated missing report and remain
eligible for ID reacquisition for 1.2 seconds. The target advertises
`kTrackIdentity`, so display selection/locking follows the software ID rather
than an RD-03D slot. Diagnostics are quiet by default; type `v` to toggle the
full link/radar/system diagnostics and `h` for help.

No MPU is required. If one is attached, its optional telemetry remains isolated
from contact association, filtering, frame motion state, and link health.

## C4001 raw-capture target

`c3-c4001-capture-rdp` is an independent interim backend for the five-pin
DFRobot C4001 SEN0609. It does not replace or remove the RD-03D component.
UART0 reads GPIO4 at the documented 9600 baud, and optional GPIO5 diagnostics
sample the sensor's `OUT` signal.

```powershell
pio run -e c3-c4001-capture-rdp
pio run -e c3-c4001-capture-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-c4001-capture-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The bounded collector accepts either `$DFHPD...*` presence reports or
`$DFDMD...*` distance/speed reports without issuing any sensor command. Type
`c` to capture another 32 reports. The attached SEN0609 has been physically
verified at 9600 baud, so this target remains fixed at 9600 and does not probe
alternate rates. C4001 output is quiet by default and logs positive presence
transitions. Monitor commands are `c` for 32 raw reports, `q` to pause raw
reports, and `v` to toggle verbose diagnostics. Follow
[`hardware/scanner-link/c4001-to-esp32-c3.md`](../../hardware/scanner-link/c4001-to-esp32-c3.md)
for safe VIN, UART, and optional OUT wiring.

## C4001 live presence target

`c3-c4001-live-rdp` replaces raw logging with strict presence normalization
and publishes canonical `RadarFrame` packets over the proven C3-to-S3 link.
It remains fixed at 9600 baud and sends no commands to the sensor.

```powershell
pio run -e c3-c4001-live-rdp
pio run -e c3-c4001-live-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-c4001-live-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Only exact `$DFHPD,0, , , *` and `$DFHPD,1, , , *` reports are accepted.
Malformed reports and `$DFDMD` distance-mode reports are counted but never
forwarded. Two consecutive equal reports confirm initial state and transitions;
subsequent valid reports refresh the frame arrival time. Published frames are
healthy, presence-only snapshots with zero contacts and `kPresence` as their
only capability. The S3 therefore renders `AREA CLEAR` or `PRES DETECT` but no
contact dots.

## C4001 live range target

`c3-c4001-range-live-rdp` handles a C4001 already producing `$DFDMD` reports.
It strictly decodes distance, signed radial velocity, and energy, applies the
two-report target-count filter, and publishes range-only canonical frames over
the existing C3-to-S3 link.

```powershell
pio run -e c3-c4001-range-live-rdp
pio run -e c3-c4001-range-live-rdp -t upload --upload-port COM_NUMBER
pio device monitor -e c3-c4001-range-live-rdp --port COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

The target sends no sensor commands and needs only C4001 TX on GPIO4 plus
power/common ground. Because C4001 has no bearing output, the S3 draws its one
contact at the measured range on the forward centerline. Large raw energy
values saturate only the canonical 16-bit signal field; they do not invalidate
an otherwise valid report.

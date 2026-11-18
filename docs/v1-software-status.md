# N203 v1 software status

Status: software complete; final hardware acceptance pending  
Updated: 2026-08-07

## Production configuration

```text
RD-03D TX -> ESP32-C3 GPIO4
ESP32-C3  -> typed COBS/CRC UART link -> ESP32-S3
ESP32-S3  -> shared renderer -> ILI9488
```

Use these firmware targets:

```powershell
pio run -e s3-scanner-display-rdp
pio run -e c3-rd03d-position-live-rdp
```

The RD-03D path is read-only. Its RX pin and C3 GPIO6 remain disconnected. The
MPU-6050 is optional telemetry and is not involved in contact generation,
association, smoothing, motion state, or display link health.

## Completed software

- shared desktop/S3 renderer and calibrated touch interaction;
- canonical presence, range, position, azimuth, velocity, confidence, and track
  identity model with validation;
- versioned COBS/CRC link, descriptor negotiation, acknowledgments, sequence
  tracking, corruption recovery, and stale/offline presentation;
- bounded RD-03D raw framing and sign-magnitude decoding at 256000 baud;
- physically verified canonical transform: native X is inverted once and
  positive Y remains forward;
- allocation-free, three-contact association with software IDs, prediction,
  position smoothing, 300 ms visible dropout hold, and 1.2-second identity
  reacquisition window;
- existing display selection and lock behavior driven by stable software IDs;
- separate capture, range-only, and position-verification diagnostic builds;
- independent C4001 diagnostic/live backends.

## Automated acceptance

Configure, build, and run the native suite:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Run only the repeatable pipeline soak:

```powershell
ctest --test-dir build -C Debug -L soak --output-on-failure
```

`n203-rd03d-soak-tests` simulates 30 minutes at 10 Hz: 18,000 complete raw
RD-03D frames pass through framing, decoding, three-contact association,
crossing trajectories, source-slot permutations, one- and two-frame dropouts,
canonical validation, wire serialization/deserialization, and the display app.
The test also keeps one contact selected and locked for the entire run.

The soak found and now guards against a negative-velocity prediction regression
that could previously replace a left-moving contact's software ID on its third
measurement.

Build the production firmware pair:

```powershell
pio run -e c3-rd03d-position-live-rdp
pio run -e s3-scanner-display-rdp
```

## Remaining hardware-only acceptance

Software completion does not substitute for these physical checks:

- verify production firmware center/right/left and approach/retreat behavior;
- select and lock a contact, then confirm the lock follows ordinary movement;
- verify exit, a sub-1.2-second re-entry, and a longer absence;
- test two people and crossing paths if the RD-03D emits multiple targets in its
  present configuration;
- run both boards and the display for at least 30 minutes while watching link,
  framing, rejection, reset, and heap counters;
- power-cycle the scanner and display independently and confirm recovery;
- verify the final shared power arrangement before enclosure installation.

If physical multi-target output is unavailable, enabling or configuring it is a
separate command-path phase. Do not connect RD-03D RX merely to experiment;
implement commands with explicit acknowledgments and documented module values.

## Deferred optional work

- physical MPU-6050 characterization and motion-quality thresholds;
- `DOPPLER WARNING`/`STABILIZING` motion-state fusion;
- touchscreen-to-scanner configuration commands;
- writable RD-03D configuration and future radar backends;
- enclosure, final power distribution, and deployment packaging.

These items are not required for the read-only v1 RD-03D position tracker.

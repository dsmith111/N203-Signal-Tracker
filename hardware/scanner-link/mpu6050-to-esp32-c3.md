# MPU-6050 to ESP32-C3 telemetry bench

Status: optional, non-blocking firmware integration implemented; physical
breakout and motion captures pending.

The MPU-6050 is a six-axis IMU: a three-axis gyroscope plus a three-axis
accelerometer. N203 uses both. The gyroscope measures rotation of the scanner
carrier, while the accelerometer supports gravity and dynamic-acceleration
estimates.

## Electrical gate

Generic MPU-6050 breakouts are not electrically identical. Some include a
regulator and I2C level handling; others expose the sensor supply directly.
Until the exact breakout is identified, use 3.3 V and do not apply 5 V to its
I2C pull-ups or either ESP32-C3 GPIO.

Power everything down before changing wiring:

```text
MPU-6050 VCC -> C3 3.3V
MPU-6050 GND -> C3 GND
MPU-6050 SDA -> C3 GPIO0
MPU-6050 SCL -> C3 GPIO1

MPU-6050 INT -> disconnected
MPU-6050 XDA -> disconnected
MPU-6050 XCL -> disconnected
```

`AD0` may remain in the breakout's default state. The firmware actively probes
both standard addresses, `0x68` and `0x69`, and validates the MPU-6050 identity
register before configuring it. If the breakout has no I2C pull-ups, add one
pull-up from SDA to 3.3 V and one from SCL to 3.3 V, normally 4.7 kohm each. Do
not add another pair until the breakout has been checked for onboard pull-ups.

Mount the MPU-6050 and RD-03D rigidly to the same carrier. A loosely wired IMU
or one attached only to the desk measures the wrong motion for later radar
quality decisions.

## Firmware behavior

MPU-6050 support is compiled into the normal C3 targets, including
`c3-rd03d-capture-rdp`; a separate IMU build is not required. The service:

- probes `0x68` and `0x69` without blocking the UART services;
- selects a 100 Hz sample rate, approximately 42-44 Hz digital low-pass filter,
  +/-2 g accelerometer range, and +/-250 degrees/second gyro range;
- collects 200 startup samples for gyro bias estimation;
- low-pass filters acceleration to estimate gravity;
- reports bias-corrected angular-rate magnitude and dynamic acceleration;
- retries automatically if the IMU is absent or unplugged;
- publishes telemetry only and does not change the display motion state yet.

Keep the complete scanner carrier stationary for roughly two seconds after the
log enters `state=calibrating`. A successful startup resembles:

```text
[i2c] device address=0x68
[imu] state=configuring address=0x68
[imu] state=calibrating address=0x68
[imu] keep scanner carrier stationary for 200 startup samples
[imu] state=ready address=0x68
[imu] bias complete gx=... gy=... gz=... dps; telemetry only, display motion state unchanged
```

The once-per-second diagnostic contains the measured axes and derived values:

```text
[imu] state=ready address=0x68 hz=... samples=... errors=0/0 disconnects=0 reconnects=0 age_ms=... calibrated=yes accel=... g gyro=... dps gyro_mag=... dynamic_g=...
```

An absent device remains in `state=probing`; a removed device changes to
`state=disconnected` and is reprobed once per second. Neither condition stops
RD-03D capture, scanner-link negotiation, or USB diagnostics.

## Build, upload, and monitor

```powershell
pio run -e c3-rd03d-capture-rdp
pio run -e c3-rd03d-capture-rdp -t upload --upload-port C3_COM_NUMBER
pio device monitor -e c3-rd03d-capture-rdp --port C3_COM_NUMBER --baud 115200 --dtr 0 --rts 0
```

Record telemetry for stationary-table, normal walking, scan posture while
walking, slow translation, slow and rapid rotation, shaking, and hinge
deployment. Those measurements—not assumed thresholds—will define the later
tracking/warning/stabilizing state machine.

# ESP32-C3 16-pin scanner board

Status: pin layout supplied; physical upload and duplex C3-to-S3 link proven;
RD-03D, C4001, and optional MPU-6050 assignments implemented.

The supplied orientation is USB-C at the top with the component side facing
the viewer:

```text
left, top to bottom       right, top to bottom
------------------       --------------------
GPIO5                    5V
GPIO6                    GND
GPIO7                    3.3V
GPIO8                    GPIO4
GPIO9                    GPIO3
GPIO10                   GPIO2
GPIO20                   GPIO1
GPIO21                   GPIO0
```

Buttons are BOOT on the left and RESET on the right. This matches the common
16-pin ESP32-C3 SuperMini layout. The firmware uses the conservative
`esp32-c3-devkitm-1` 4 MB build definition rather than depending on an
unverified vendor-specific board manifest.

## Pin assignment

```text
GPIO21  scanner-link TX  -> S3 GPIO41 RX
GPIO20  scanner-link RX  <- S3 GPIO42 TX

GPIO4   radar RX <- selected radar TX
GPIO5   C4001 OUT input; spare for RD-03D
GPIO6   radar command TX -> selected radar RX (later; disconnected for capture)

GPIO0   reserved MPU-6050 SDA
GPIO1   reserved MPU-6050 SCL

GPIO3   spare
GPIO7   spare
GPIO10  spare
```

The link is routed through UART1; the later radar probe uses UART0. Native USB
diagnostics remain on the ESP32-C3 USB Serial/JTAG peripheral rather than
either hardware UART.

GPIO2, GPIO8, and GPIO9 remain unused because they are ESP32-C3 strapping
pins. GPIO9 is associated with BOOT on this layout, and GPIO8 is commonly used
by the onboard LED. GPIO18 and GPIO19 carry native USB internally and are not
exposed on this header.

## Simulated-link bench wiring

Power both boards through their own working USB connections. Join grounds but
do not join the 5 V or 3.3 V rails.

Stage A proves the one-way stream:

```text
C3 GPIO21 (TX) -- optional 470 ohm series resistor --> S3 GPIO41 (RX)
C3 GND ---------------------------------------------> S3 GND
```

Stage B enables negotiation:

```text
S3 GPIO42 (TX) -- optional 470 ohm series resistor --> C3 GPIO20 (RX)
```

Establish common ground before either signal connection. Verify each UART line
idles near 3.3 V and never apply 5 V to a GPIO. Keep the two UART wires short
and separate from the ILI9488 SPI bundle.

The simulated-link bench has passed. See
[`rd-03d-to-esp32-c3.md`](rd-03d-to-esp32-c3.md) before attaching the RD-03D.
The independent C4001 diagnostic path is documented in
[`c4001-to-esp32-c3.md`](c4001-to-esp32-c3.md). Follow
[`mpu6050-to-esp32-c3.md`](mpu6050-to-esp32-c3.md) before attaching the exact
MPU-6050 breakout to GPIO0/GPIO1.

## Expected diagnostics

The C3 USB log should show:

```text
[boot] ... protocol=v2 session=...
[pins] link_rx=20 link_tx=21 ...
[i2c] scan complete ...
[imu] state=ready address=0x68 ...
[link] HelloAck accepted ...
[link] connected=yes ...
[radar] mode=simulated ...
```

The S3 USB log should show the `c3-sim-v0.1` / `simulated` descriptor,
`connected=yes`, and approximately ten new frames per second. Rejection, gap,
duplicate, and reset counters should remain zero after the initial connection.

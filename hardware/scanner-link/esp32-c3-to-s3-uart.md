# ESP32-C3 to ESP32-S3 scanner link

Status: assignments and physical duplex `Hello`/`HelloAck` bench complete.

## Display-node assignment

```cpp
constexpr int kScannerUartRx = 41;
constexpr int kScannerUartTx = 42;
constexpr std::uint32_t kScannerUartBaud = 115200;
```

GPIO41 and GPIO42 are exposed, non-strapping pins on the documented S3 board.
They do not overlap the ILI9488/XPT2046 bus or GPIO19/20 native USB. USB CDC
therefore remains available for firmware upload and diagnostics while the
hardware UART carries scanner frames.

## Wiring contract

```text
Scanner C3 TX (3.3 V) -> Display S3 GPIO41 (RX)
Scanner C3 RX (3.3 V) <- Display S3 GPIO42 (TX, v2 HelloAck return path)
Scanner C3 GND         -> Display S3 GND
```

TX-to-RX and common ground are sufficient for a one-way framing smoke test.
The return wire is required to complete the v2 `Hello`/`HelloAck` negotiation.

Rules:

- Both UART signal levels must be 3.3 V logic.
- Never apply 5 V to either UART GPIO.
- Establish common ground before connecting signals.
- Do not connect the boards' 5 V rails together unless a later power design
  explicitly establishes one shared supply and its current budget.
- Keep initial bench wires short and separate from the display SPI bundle.
- Confirm the C3's chosen pins are not boot straps, flash pins, USB pins, or
  occupied by the radar module before wiring them.

The byte-level contract is defined in
[`docs/radar-link-v2.md`](../../docs/radar-link-v2.md).
The C3 board assignment and exact cross-wiring are recorded in
[`esp32-c3-supermini.md`](esp32-c3-supermini.md).

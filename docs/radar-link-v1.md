# N203 radar link protocol v1

Status: superseded by [`radar-link-v2.md`](radar-link-v2.md). This document is
retained as the record of the first one-way display bench protocol.

The radar link carries canonical `RadarFrame` snapshots from the scanner node
to the display node. Version 1 is a one-way 3.3 V UART data path; it does not
carry drawing commands or display interactions.

## UART configuration

```text
Baud:       115200
Data bits:  8
Parity:     none
Stop bits:  1
Flow:       none
Byte order: little-endian
Rate:       normally 10 complete frames per second
```

At the maximum eight contacts, a packet is 154 bytes and occupies about
13.4 ms on a 115200 baud 8-N-1 link. The display drains the UART continuously
but retains only the newest complete snapshot.

## Packet envelope

| Offset | Size | Field | Value |
|---:|---:|---|---|
| 0 | 4 | Sync | ASCII `N203` (`4E 32 30 33`) |
| 4 | 1 | Version | `1` |
| 5 | 1 | Message type | `1` = radar frame |
| 6 | 2 | Payload length | 16 through 144 |
| 8 | N | Payload | Frame header followed by contact records |
| 8+N | 2 | CRC | CRC-16/CCITT-FALSE |

The CRC uses polynomial `0x1021`, initial value `0xFFFF`, no reflection, and no
final XOR. It covers bytes from `version` through the end of the payload; the
four sync bytes and the CRC field itself are excluded. The standard
`123456789` check value is `0x29B1`.

## Radar-frame payload

| Payload offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Sequence number |
| 4 | 4 | Scanner-node timestamp in milliseconds |
| 8 | 4 | Capability flags |
| 12 | 1 | Radar health |
| 13 | 1 | Measurement state |
| 14 | 1 | Contact count, 0 through 8 |
| 15 | 1 | Reserved, must be zero |
| 16 | 16 each | Contact records |

Payload length must be exactly `16 + contact_count * 16`.

The scanner timestamp belongs to the scanner node's clock domain. The display
retains it for diagnostics but determines link staleness from the local time at
which a valid packet arrives.

### Contact record

| Record offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Track ID |
| 1 | 1 | Confidence |
| 2 | 2 | Valid-field mask |
| 4 | 2 | X position, signed millimetres |
| 6 | 2 | Y position, signed millimetres |
| 8 | 2 | Range, unsigned millimetres |
| 10 | 2 | Azimuth, signed centidegrees |
| 12 | 2 | Radial velocity, signed millimetres/second |
| 14 | 2 | Signal strength |

Signed values use 16-bit two's-complement representation. A numeric field is
only meaningful when its corresponding valid-field bit is set.

## Enum and flag values

Radar health:

```text
0 unknown
1 healthy
2 degraded
3 offline
```

Measurement state:

```text
0 initializing
1 scanning
2 range-only
3 presence-only
4 stale
5 offline
```

Capability bits:

```text
bit 0 presence
bit 1 range
bit 2 position
bit 3 azimuth
bit 4 radial velocity
bit 5 stable track identity
bit 6 multiple contacts
```

Contact valid-field bits:

```text
bit 0 position X/Y
bit 1 range
bit 2 azimuth
bit 3 radial velocity
bit 4 signal
bit 5 confidence
```

## Receiver behavior

The receiver searches for the sync bytes, validates version, type, length, CRC,
enum values, contact count, and the canonical frame contract. A rejected packet
never changes the displayed snapshot. Parsing resumes at the next sync word.

Sequence discontinuities are diagnostic only. A forward jump counts missing
frames; a backward jump counts a scanner restart or out-of-order frame. CRC,
format, payload, sequence-gap, and superseded-frame counters are reported over
the S3's USB diagnostics channel.

## Historical scanner-node sender contract

The v1 sender constructed a canonical `RadarFrame` and wrote one sync-framed
packet per snapshot. That API and runtime decoder were removed after v2 gained
equivalent recovery coverage. New senders must use the typed v2 encoders in
[`radar-link-v2.md`](radar-link-v2.md).

Sensor-specific RD-03D, LD2420, or other parsing belongs before this boundary.
The wire format remains sensor-independent.

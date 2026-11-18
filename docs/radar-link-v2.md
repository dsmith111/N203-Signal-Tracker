# N203 scanner link protocol v2

Status: current prototype protocol  
Transport: 115200 baud, 8-N-1 UART, 3.3 V logic

The link carries scanner identity and canonical `RadarFrame` snapshots between
the ESP32-C3 scanner node and ESP32-S3 display node. All multibyte integers are
unsigned little-endian unless a canonical field is explicitly signed.

## Packet framing

Each decoded packet has this layout:

```text
offset  size  field
0       1     protocol version (2)
1       1     packet type
2       2     payload length
4       4     packet sequence
8       4     sender uptime in milliseconds
12      N     typed payload
12+N    2     CRC-16/CCITT-FALSE
```

The CRC covers the header and payload, starting at offset 0. Parameters are
polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR. The
check value for `123456789` is `0x29B1`.

The complete decoded packet, including CRC, is COBS encoded and followed by a
single `0x00` delimiter. Encoded packet data therefore contains no zero byte.
Endpoints discard an incomplete or malformed packet at the next delimiter and
resume with the following packet. Buffers are bounded by
`kMaximumPacketSize`; an overlong packet is ignored through its delimiter.

## Packet types

```text
1  Hello
2  HelloAck
3  RadarFrame
```

Types for commands, command acknowledgments, diagnostics, and heartbeats are
reserved for later revisions and are not accepted by v2 implementations.

### Hello payload (48 bytes)

```text
offset  size  field
0       4     scanner session/boot identifier
4       4     canonical RadarCapability flags
8       4     maximum range in millimetres; 0 if unspecified
12      2     horizontal field of view in centidegrees; 0 if unspecified
14      1     maximum contacts (0 through 8)
15      1     reserved, must be zero
16      16    scanner firmware identity, zero-padded bytes
32      16    active radar type, zero-padded bytes
```

The scanner sends `Hello` after boot and repeats it until a matching
`HelloAck` arrives. It should continue a slower periodic `Hello` after
negotiation so an independently restarted display can rediscover the active
session. The display accepts a changed session without requiring either board
to reset and resets its frame-sequence baseline for that session.

### HelloAck payload (8 bytes)

```text
offset  size  field
0       4     scanner session identifier being acknowledged
4       1     accepted protocol version (2)
5       3     reserved, must be zero
```

The S3 sends an acknowledgment for every valid `Hello`, including a repeated
Hello, so a lost return packet cannot permanently stall negotiation.

### RadarFrame payload

The payload has a 20-byte frame header followed by zero through eight 16-byte
contacts:

```text
offset  size  field
0       4     frame sequence
4       4     scanner/sensor timestamp in milliseconds
8       4     canonical RadarCapability flags
12      1     RadarHealth
13      1     MeasurementState
14      1     PresenceState
15      1     MotionState
16      1     motion quality, 0 through 100
17      1     contact count
18      2     reserved, must be zero
20      ...   contacts
```

Each contact is:

```text
offset  size  field
0       1     contact id
1       1     confidence
2       2     valid-field flags
4       2     signed x millimetres
6       2     signed y millimetres
8       2     range millimetres
10      2     signed azimuth centidegrees
12      2     signed radial velocity millimetres/second
14      2     signal value
```

The receiver validates enum ranges, reserved bytes, contact count, unique
contact IDs, capability/field combinations, explicit presence semantics, and
motion quality before publishing a frame. A rejected packet never replaces the
currently displayed snapshot.

## Sequence and recovery behavior

Packet and frame sequence values wrap naturally at 32 bits. A forward delta of
one is in order; a larger forward delta records missing values; zero records a
duplicate; a delta in the backward half of the sequence space records a
restart. Duplicate frames are not replayed to the display.

The S3 continuously drains bounded UART input and keeps only the newest valid
frame from each polling pass. Display staleness uses the S3's local arrival
time, never the C3 timestamp. Resetting either endpoint, disconnecting the
cable, a partial packet, garbage, bad COBS, bad CRC, or an invalid payload must
recover without resetting the other endpoint.

## Maximum sizes

With eight contacts:

```text
RadarFrame payload       148 bytes
decoded packet           162 bytes
maximum COBS data        163 bytes
packet plus delimiter    164 bytes
```

Both implementations use fixed-size buffers and perform no heap allocation per
packet.

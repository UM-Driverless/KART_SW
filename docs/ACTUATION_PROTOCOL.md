# Actuation Protocol

## Overview
The Orin sends actuator targets to the ESP32 over USB serial (UART). This document
defines the current wire format and a planned, more robust version with checksum.

Reference firmware repo: https://github.com/UM-Driverless/kart-medulla

## Version 1 (current)
Fixed 4-byte frame, no checksum. Steering uses signed 8-bit (s8): negative = right,
positive = left, 0 = straight.

| Byte | Name      | Type   | Notes                          |
|------|-----------|--------|--------------------------------|
| 0    | START     | u8     | 0xAA                           |
| 1    | STEER     | s8     | -127..127 (0 = straight)       |
| 2    | THROTTLE  | u8     | 0–255                          |
| 3    | BRAKE     | u8     | 0–255                          |

Scaling (current):
- `STEER`: `steering * 127.0` clamped to [-127, 127]
- `THROTTLE`: `acceleration` if > 0, scaled `* 255`
- `BRAKE`: `-acceleration` if < 0, scaled `* 255`

Safety expectation on ESP32:
- If no new frame arrives within a timeout, apply full brake, zero steering, zero throttle.

## Version 2 (planned)
Add sequence + checksum for robustness and loss detection. Also switch steering to
signed 8-bit (s8): negative = right, positive = left, 0 = straight.

| Byte | Name      | Type   | Notes                          |
|------|-----------|--------|--------------------------------|
| 0    | START     | u8     | 0xAA                           |
| 1    | SEQ       | u8     | increments each frame          |
| 2    | STEER     | s8     | -127..127 (0 = straight)       |
| 3    | THROTTLE  | u8     | 0–255                          |
| 4    | BRAKE     | u8     | 0–255                          |
| 5    | CRC8      | u8     | CRC-8 over bytes 0–4           |

Notes:
- Use CRC-8 (poly 0x07) or XOR if simplicity is required.
- Requires firmware update in `kart-medulla` to parse V2 frames.

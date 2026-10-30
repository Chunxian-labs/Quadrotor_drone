# Radio Protocol

The flight controller and handheld controller use the same fixed 32-byte payload format.

## Radio settings

| Setting | Value |
| --- | --- |
| Transceiver | NRF24L01 |
| RF channel | 2 (2402 MHz) |
| Data rate | 2 Mbps |
| Address width | 5 bytes |
| TX/RX address bytes | `11 22 33 44 55` |
| Payload width | 32 bytes in both directions |
| CRC | Enabled, 1 byte |
| Automatic acknowledgment | Enabled |
| Retransmission configuration | 250 us interval, up to 3 retries |

Both devices return to receive mode after a transmission. Application telemetry is sent as a separate `0x12` packet; it is not an ACK payload.

## RC command: controller to aircraft

| Byte offset | Type | Field |
| --- | --- | --- |
| 0 | uint8 | `0x10`: command; `0x11`: command requesting telemetry |
| 1 | int8 | LH: yaw input, -100 to 100 |
| 2 | int8 | LV: throttle / vertical-speed input, -100 to 100 |
| 3 | int8 | RH: roll input, -100 to 100 |
| 4 | int8 | RV: pitch input, -100 to 100 |
| 5 | uint8 | Key code: 0 for none, 1 to 4 for flight commands |
| 6-31 | bytes | Reserved; cleared by the handheld firmware |

The handheld requests telemetry in every command (`0x11`). Its transmit flag is scheduled every 20 ms; K2 also requests an immediate transmission from the main loop. The aircraft accepts `0x10` and `0x11`, validates joystick ranges, and attaches a local FreeRTOS reception timestamp before queuing the command. This timestamp is internal and is not transmitted.

## Telemetry: aircraft to controller

| Byte offset | Type | Field / unit |
| --- | --- | --- |
| 0 | uint8 | Packet ID: `0x12` |
| 1 | uint8 | Mode: 0 = stabilization, 1 = altitude hold |
| 2-3 | int16 | Collective throttle command |
| 4-7 | float32 | Battery voltage, V |
| 8-11 | float32 | Roll, degrees |
| 12-15 | float32 | Pitch, degrees |
| 16-19 | float32 | Yaw, degrees |
| 20-23 | float32 | Corrected height observation, cm |
| 24 | uint8 | Latched undervoltage flag |
| 25 | uint8 | Run state: 0 = STOP, 1 = READY, 2 = RUNNING |
| 26-31 | bytes | Reserved; zero-filled |

Multi-byte values use the STM32 little-endian representation; floats are 32-bit IEEE 754. Fields are copied at explicit offsets with `memcpy`, rather than transmitting a padded C structure.

Telemetry reports the latest published flight snapshot and may precede execution of the requesting command. The height field is the corrected height observation, rather than the complementary estimator's internal height state. Hardware ACK success acknowledges radio reception; the displayed `CMD` label records a local command, while telemetry `run_state` reports aircraft state.

## User commands

| Key | Aircraft action |
| --- | --- |
| K1 | Enter READY, stop motors, clear vertical target, request clearing of undervoltage latch |
| K2 | Enter STOP and stop motors |
| K3 | Select stabilization mode |
| K4 | Select altitude-hold mode |
| K9 | Change the handheld OLED page; not transmitted as a flight command |

After READY, an LV value above 10 starts flight control. In stabilization mode LV sets manual collective throttle; in altitude mode it sets vertical speed (`LV / 2`, cm/s). RH sets roll (`RH / 5`, degrees), RV sets pitch (`-RV / 5`, degrees), and LH controls yaw-target integration.

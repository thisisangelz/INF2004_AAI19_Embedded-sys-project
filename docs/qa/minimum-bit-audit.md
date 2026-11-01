# Minimum-bit audit register

Record application fields here when they are audited. Do not treat C bit-fields as portable wire layouts, and do not claim savings without map, size, or protocol measurements.

| Field | Source | Valid range / precision | Minimum logical bits | Physical storage | Encoding / invalid value | Owner / protection | Overflow handling | Measured before / after |
|---|---|---|---:|---:|---|---|---|---|
| RSSI sample | `getting_there/packed_bits.h` | -126..-1 dBm; -127 missing | 7 | packed 7-bit slot | magnitude; 127 missing | callback/main handover | caller validates input domain | 12 samples = 11 bytes |
| Fault set | `getting_there/faults.*` | 12 independent faults | 12 | `uint16_t` | one bit per fault | main/callback; IRQ-masked updates | compile-time count assertions | not yet map-measured |
| BLE state | `getting_there/payload_client.c` | defined state enum through closing | 5 | shared `uint32_t` field | unsigned enum | main/BTstack; IRQ-masked updates | compile-time width/layout assertions | not yet map-measured |

For each proposed change, add boundary and round-trip tests and attach `arm-none-eabi-size` or linker-map evidence. Audit timestamp arithmetic using modular unsigned subtraction and include the terminal counter value when calculating width.


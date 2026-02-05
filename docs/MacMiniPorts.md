# Mac Mini Ports

Location IDs represent **USB 2** devices connected directly to the port and are *not* accurate for USB 3 devices or USB/Thunderbolt hubs.

## Top-Down

| Front Ports      | Back Ports               |
| ---------------- | ------------------------ |
| USB (0x02120000) | Thunderbolt (0x03100000) |
| USB (0x02110000) | Thunderbolt (0x00100000) |
|                  | Thunderbolt (0x01100000) |
|                  | HDMI                     |
| Status Light     | Ethernet                 |
| Headphone Jack   | AC Input                 |

## Bottom-Up

| Front Ports      | Back Ports               |
| ---------------- | ------------------------ |
| Headphone Jack   | AC Input                 |
| Status Light     | Ethernet                 |
|                  | HDMI                     |
|                  | Thunderbolt (0x01100000) |
| USB (0x02110000) | Thunderbolt (0x00100000) |
| USB (0x02120000) | Thunderbolt (0x03100000) |

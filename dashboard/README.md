# MQTT Dashboard Protocol

Default topic root is `robot/se09-car/v3`. Use this dashboard with the integrated firmware. The supplied progress ZIP uses a different v2 message layout. Change `MQTT_CLIENT_ID` or `MQTT_TOPIC_ROOT` in the car's secrets header and pass the matching `--topic` to `dashboard/app.py`.

| Topic suffix | Direction | QoS | Retained | Binary payload |
| --- | --- | --- | --- | --- |
| `/state` | Car to clients | 0 | No | 13-byte status snapshot |
| `/online` | Car to clients | 0 | Yes | Byte 1 on connection, byte 0 in the broker's last will |
| `/command` | Client to car | 1 | No | Byte 1 requests stop. Byte 2 requests current status and route table |
| `/routes/set` | Client to car | 1 | Yes | Complete 10-byte route table |
| `/routes/state` | Car to clients | 1 | Yes | Last route table applied by the car |

Periodic status arrives at most every two seconds while moving and every five seconds while idle. Major state and event changes can publish sooner, with a minimum interval for change-triggered messages. The dashboard treats telemetry older than eight seconds as offline.

The route table is 80 bits, little-endian. The low two bits are version 1. A through Z each occupy three bits starting at bit 2. Actions are 0 unassigned, 1 left, 2 right, 3 straight, 4 U-turn. The car rejects any other length, version or action as a whole. The dashboard only reports success after the returned table matches the update. The compiled starting map follows the progress firmware: A straight, B U-turn, C left and F right. The rest are unassigned. Set `BARCODE_PROGRESS_MAP=0` at build time to use the example in the write-up as the starting map.

Use the dashboard for single-letter edits. A direct MQTT client can use `encode_routes` in `protocol.py` to construct a full replacement, then publish the bytes to the retained set topic. Clear a letter by writing it as action zero in that replacement. A route stays armed once accepted by the controller, even if a new table arrives before the junction.

The status layout is documented by bit offset in [DATA_REPRESENTATION.md](../testing/DATA_REPRESENTATION.md). The dashboard uses a broker account with publish access to `/routes/set` and `/command`. The car account needs to subscribe to those topics and publish the other three. Keep the broker on the course LAN and enable broker persistence if mappings should survive a broker restart.

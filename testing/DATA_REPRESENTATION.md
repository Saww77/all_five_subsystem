# Application Data Representation

The firmware packs flags and bounded state. Shared snapshots, the route table and the pending encoder word are copied with interrupts masked on the single control core. The PWM writer and emergency override cannot race on that core. C bit-field layout is internal. MQTT uses explicit little-endian bit positions.

| Firmware value | Supported range | Representation |
| --- | --- | --- |
| Encoder delta | 0–100 edges per wheel per control interval | 7 bits per wheel and one overflow bit in a single 16-bit ISR handoff |
| Position and travelled distance | Signed wheel positions and modular unsigned odometry in µm | Native 32-bit accumulators for rollover and sufficient run length |
| Goal wheel speed | −180 to 180 mm/s | Signed 9-bit field |
| Encoder direction | Forward or reverse | 1 bit per wheel |
| Filtered wheel speed | Q8 signed encoder estimate, bounded by edge limit and minimum control interval | Signed 22-bit field |
| PID error, integral and PWM | Error −127 to 127, integral −18500 to 30000, duty 0–900 counts | Signed 8, signed 16 and 10-bit hardware range |
| Reflectance samples | 0–4095 for each ADC | Three 12-bit fields in one transient word |
| Line error and flags | Signed normalised error, present/junction/scan/calibrated | Signed 11 bits and four 1-bit flags |
| Barcode | 0 or A–Z, 29 runs of 1–22 mm, sequence wraps | 5-bit letter, 5-bit run count, 5-bit run slots, 8-bit event ID |
| Raw echo and centred range | Echo 90–29000 µs, range 0–4095 mm with zero denoting no return | 15-bit physical echo range in a 16-bit pulse value, 12-bit range, two validity flags and a 32-bit trigger timestamp |
| Obstacle profile | Lateral −250 to 250 mm, nearest 0–540 mm, width 0–1023 mm | Signed 9-bit lateral, 10-bit nearest, 10-bit width |
| Obstacle clearance, leg lengths and plan | 0–511 mm, five possible actions | 9 bits per distance, 3 bits for the plan |
| IMU | Native high-resolution signed 12-bit acceleration, signed 16-bit magnetometer | Convert to signed processing units for calibration, projection and integration |
| Terrain event and hump | Eight classes, max peak 0–1023 mm | 3-bit event and 10-bit mm peak |
| Phase, faults and event IDs | Eight mission states, eight fault conditions, five route actions, event flags | 3-bit mission and action IDs, one shared 8-bit fault mask, kernel flag bits |
| MQTT command | 1 Stop or 2 Report | 2 bits in a one-byte MQTT payload |
| Route configuration | A–Z, five actions, version | 26 × 3 bits and a 2-bit version. Exactly 10 bytes |

Positions, timestamps and filter accumulators retain their native working widths where needed for arithmetic. Sequence numbers are modular freshness tokens. Status fields that share a C allocation unit can have alignment padding on the target ABI.

## MQTT state snapshot

Offsets are zero-based. Bit zero is the least significant bit of byte zero. Signed values use two's complement.

| Offset | Bits | Field |
| ---: | ---: | --- |
| 0 | 4 | Protocol version 3 |
| 4 | 3 | Mission phase, Setup through Stopped |
| 7 | 3 | Terrain event, Still through Impact |
| 10 | 5 | Barcode, 0 unknown, 1–26 A–Z |
| 15 | 3 | Obstacle plan. In IR setup, 0 unspecified, 1 white target, 2 black target, 3 sampling, 4 insufficient contrast |
| 18 | 1 | Line in view |
| 19 | 1 | Magnetometer contributes to yaw rate |
| 20 | 8 | Stop, impact, motor/encoder, IMU, line, path, timing and ultrasonic signal in that order |
| 28 | 24 | Distance travelled since start in mm |
| 52 | 12 | Forward range in mm, 0 for no return |
| 64 | 10 | Highest completed hump in mm |
| 74 | 10 | Last obstacle's estimated width in mm |
| 84 | 10 | Signed forward speed in mm/s |
| 94 | 10 | Signed yaw rate in deg/s |

## RAM and message size comparison

Both images were linked for RP2040 in the same environment with Pico C SDK 2.3.1 and `SMP=0 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1`. The baseline was the supplied SE09 progress ZIP. Values are from `arm-none-eabi-size`.

| Firmware | Data | BSS | Static RAM |
| --- | ---: | ---: | ---: |
| SE09 progress snapshot | 100 | 78,648 | 78,748 bytes |
| Integrated firmware | 96 | 78,260 | 78,356 bytes |

The integrated image uses 392 fewer bytes of static RAM in this build. This comparison covers the integrated change, including task ownership and serialization, and does not isolate the effect of bit packing. The five application task stacks request 1,600, 1,280, 1,200, 960 and 512 bytes respectively. Free heap and stack high-water measurements require an on-car run.

| Application message | Progress snapshot | Integrated firmware |
| --- | ---: | ---: |
| Main controller MQTT snapshot | 31 bytes | 13 bytes |
| MQTT command | 1 byte | 1 byte |
| Full retained A–Z route mapping | Unavailable | 10 bytes |
| Latest state transfer to MQTT | Copied application frame plus route ID and kernel message-buffer overhead | One 20-byte synchronized snapshot. No message queue |
| Encoder ISR handoff | A packed edge sample | 2 bytes |
| Line, terrain and forward range snapshots | Separate data paths in the progress snapshot | 16, 4 and 8 bytes respectively |

The integrated status frame omits raw reflectance and debug counters and adds the course-facing terrain, distance, and obstacle fields. Sizes are application payloads before MQTT headers. Host tests cover round-trip protocol fields, letter remapping at junctions, scanner and motion bounds. Physical timing and accuracy checks are listed in [the verification plan](README.md).

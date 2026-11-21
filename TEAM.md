# Hardware and Subsystem Interfaces

The project uses the course template's Pico W BSP and its interrupt-backed ADC and I²C drivers. Motor and encoder pins match the team's tested progress image. Its right encoder A is on GP0, while the Week 6 table lists GP3. The Week 6 table supplies the IR, I²C, ultrasonic and servo assignments.

| Signal | GPIO | Owner and connection |
| --- | --- | --- |
| Left motor | GP8, GP9 | Buddy 2. Robo Pico M1A/M1B, 20 kHz PWM |
| Right motor | GP10, GP11 | Buddy 2. Robo Pico M2A/M2B, 20 kHz PWM |
| Left encoder A | GP2 | Buddy 2. Rising-edge GPIO interrupt |
| Right encoder A | GP0 | Buddy 2. Rising-edge GPIO interrupt. USB CDC leaves GP0 available |
| Encoder B phases | GP3 left, GP1 right | Input with pull-up as in the progress code. Speed and position use channel A and commanded direction |
| Reflectance | GP26, GP28 | Buddy 3. Left and right line ADC inputs. Keep the GP28 VBAT solder jumper open |
| Barcode reflectance | GP27 | Buddy 3. Separate ADC reader mounted on the left, over the print |
| Optional IR comparator outputs | GP6, GP7 | Week 6 lists left/right digital outputs here. This firmware reads the analog channels and preserves the template's unused I²C1 pin assignment |
| LSM303DLHC | GP4, GP5 | Buddy 4. I²C0 SDA/SCL. Connect external pull-ups to 3.3 V as required by the breakout |
| Ultrasonic | GP16, GP17 | Buddy 5. Trigger output and interrupt-timestamped echo input. Divide or level-shift 5 V ECHO to Pico-safe levels |
| Servo | GP12 | Buddy 5. 50 Hz PWM. Logical 90° is ahead and larger angles are left |
| Heartbeat LED | GP19 | Buddy 1. Same optional external LED as the progress image |
| Board buttons | GP20, GP21 | Built-in switches. GP20 captures IR during setup and stops the car after setup. GP21 starts a calibrated car. Active low |
| Pico W radio | Internal | Buddy 1. CYW43 and raw lwIP |
| Debug | USB CDC | Calibration and startup diagnostics |

## Priorities and data ownership

Smaller micro T-Kernel priority numbers run first. The shipped profile uses one core so short interrupt-masked copies are sufficient for shared values.

| Priority | Owner | Work |
| --- | --- | --- |
| IRQ 0 | GPIO and trigger timer | Signal calibration or apply the brake on GP20, capture wheel and echo edges, and end the 10 µs trigger pulse |
| IRQ 2 | BSP ADC and I²C | Complete conversions and transfers |
| 3 | Vehicle controller and Buddy 2 | Sole software owner of motion requests and PWM decisions. Handles stop and range events and runs a 10 ms control release |
| 4 | Buddy 4 terrain | 10 ms interrupt-backed IMU sampling. Calibrates attitude, classifies motion, integrates grade over measured wheel distance and triggers impact braking |
| 5 | Buddy 5 sonar | One scan request at a time, interrupt-captured echo, timeout, servo settling and adaptive scan |
| 6 | Buddy 3 line and barcode | 5 ms release, reads all three channels, publishes one coherent line snapshot and a sequence-tagged barcode |
| 17 | Buddy 1 heartbeat LED | Toggles GP19 on its 250 ms timer release |
| 18 | USB console | Boot and calibration messages |
| 20 | Buddy 1 Wi-Fi/MQTT | CYW43 and lwIP service, connection recovery, bounded publishing and route configuration |

Cyclic releases originate from the kernel timer interrupt. Tasks wait on kernel flags or transfer completion. The GPIO stop bypasses the application schedule and asserts a motor output override. The controller latches safety faults, and a 200 ms hardware watchdog resets the system if control servicing stops. The Wi-Fi driver's existing periodic service runs below all application sensing.

## Controller states

The controller transitions through IR Setup and Ready before allowing Follow. Follow can arm a barcode maneuver, hand off to Junction, or stop on an obstacle and request Scan. Scan may resume, stop, reverse once and rescan, or start Detour. Detour checks the front beam after each heading change, searches for the original line, centres the axle over it, and enters Reacquire. Reacquire requires consecutive line samples and bounds its heading search. Safety faults lead to Stopped. The motors stay braked until local start requirements are met.

| Interface | Responsibility |
| --- | --- |
| `motion_move`, `motion_turn`, `motion_velocity`, `motion_stop` | Nonblocking signed motion API. Controller owns sequencing. Interrupt-fed encoders close both wheel PID loops |
| `line_sample`, `line_calibrate`, `line_read_barcode` | Timestamped optical input, line position/confidence, wide junction and the latest barcode event |
| `terrain_sample`, `terrain_begin_run` | Peak hump, motion event, yaw estimate and calibration readiness |
| `sonar_range`, `sonar_scan`, `sonar_take_profile` | Centred range, scan request, completed obstacle profile and bounded plan |
| `controller_status`, `car_stop_command`, `route_config_apply` | Copy telemetry, latch a stop and atomically replace the letter-action table |

The LSM303DLHC has an accelerometer and magnetometer. Its yaw rate is inferred from heading changes and compared with encoder yaw. The controller falls back to encoder yaw when magnetic readings are inconsistent. Hump height is an estimate from chassis pitch and travelled distance and needs comparison against measured test humps.

A centred echo within 250 mm stops following. Once measured speed and yaw settle, the coarse scan checks 30°, 60°, 90°, 120° and 150°. The finer pass samples in 5° steps around the nearest sector. Returns are projected into forward and lateral bounds with an allowance for beam width. A quiet result leaves that sector unobserved. When both sides are unobserved, the planner prefers a geometrically viable left route and requires a fresh centred check after each pivot. The profile estimates only the visible obstacle surfaces. A blocked detour brakes, and an initial reverse is limited to one attempt on recently traversed track.

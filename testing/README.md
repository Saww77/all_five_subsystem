# Verification and Bench Plan

## Automated checks

Run `sh testing/host/run.sh` on Linux or WSL. The tests exercise production C logic for Code 39, route and telemetry packing, hump integration, profiling, encoder/PID control and controller transitions. Test traces include all four A/Z samples scaled to 141 mm and the A–Z symbol set in both scan directions. Regressions check constant slow speed through barcodes, steering deadband and slew, stop-before-scan, rejection of pre-turn range samples, distinct line samples during recovery and braking on invalid echoes. The host harness supplies simulated motor, RTOS and GPIO interfaces.

Run `python testing/host/dashboard_checks.py` with Paho and local Mosquitto installed. This starts a real loopback broker, checks the HTTP dashboard, sends binary telemetry from a simulated car, changes A and X actions, verifies the mapping acknowledgment and retained replay, and tests the nonretained stop message.

Default firmware verification uses the Arm cross compiler and Pico C SDK 2.3.1. Run `arm-none-eabi-size` on the linked ELF to reproduce the static RAM measurement in [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).

`TEST=1` holds both motor outputs in their hardware brake state and runs the sonar sweep when GP21 is pressed after setup. Use it to confirm servo direction and profile estimates before using the driving image.

On 21 November 2026, all three host C test programs passed with warnings treated as errors. The dashboard test passed against a local Mosquitto broker. Arm GNU Toolchain 15.2 and Pico C SDK 2.3.1 linked the MQTT, offline driving and motor-inhibited bench images. The pin audit matched the supplied ZIP's motor, encoder, IR and I²C assignments and the GP19 heartbeat.

## Physical verification

The rows below are bench and course acceptance tests. No physical run was recorded with this implementation.

| Case | Test | Record |
| --- | --- | --- |
| S1-01 | Use the real broker. Join Wi-Fi, edit a letter, power-cycle the Pico and reconnect after turning the access point off and back on | Join time, missed heartbeats, retained route restored, displayed mapping confirms |
| S2-01 | Raise the wheels, confirm both forward signs, count 10 slow manual turns per wheel and measure loaded rollout on the floor | Left/right µm per edge, minimum stable speed and whether duty saturates |
| S2-02 | Repeat a 500 mm straight target and 90°/180° turns at least five times. Adjust wheel circumference scaling and effective track width first, then PID gains | Distance error, yaw error, repeatability, wheel speed and duty trace |
| S3-01 | Use GP20 for white/black calibration, then GP21 to start. Centre the car on the 18 mm strip with the line sensors over opposite edges. Test a straight and the tightest bend | Lateral error, overshoot and line losses at two speeds |
| S3-02 | Drive Code 39 A, F, X and Z past GP27 in both orders. Assign distinct dashboard actions and position the prints close to their junctions | Reads and correct maneuvers over at least five passes per orientation |
| S4-01 | Power up flat and stationary. Verify pitch sign by raising the nose, then drive across two humps of measured height | Pitch sign, detected ascent/descent, peak estimate error in mm |
| S4-02 | Log a slow turn and use a controlled, wheels-raised impact check. Verify latching and reset | Turn rate vs. reference, impact classification and brake response |
| S5-01 | Verify ECHO levels with a meter or scope. Aim at walls and boxes with both open sides, one blocked side and both sides blocked | Echo latency, coarse/fine positions, location, width, clearance and selected plan |
| S5-02 | Test a bypass in each direction, once with a deep object encroaching on the return leg. Force a missing echo and a blocked cross | Reacquisition before steering resumes, no stale echo, bounded stop/reverse |
| SYS-01 | Scope GP20 and motor output while the barcodes, sonar, IMU and MQTT are active | Worst interrupt-to-brake time, 5/10 ms release jitter and physical stopping distance |
| SYS-02 | Run the full course from calibration to finish with a network drop and with every required feature active | Lap time, misses, route choices, highest hump, unexpected stops and dashboard rate |

## Calibration notes

Keep the GP27 sensor over the barcode strip and clear of the 18 mm line. Verify both directions with the barcode physically under that sensor. Start with the two line sensors at the line's edges. If the car calls the entire straight a junction, move them slightly outwards to leave reflectance headroom when both see a wide marking. If both sensors read white while centred, move them inwards. Repeat calibration after changing sensor height or lighting. Measure the sensor-to-axle distance and set `IR_TO_AXLE_MM`. The approximate left offset follows the 20 mm separation from the edge of the track line in the supplied print specification.

Measure wheel pulses and loaded distance before changing PID gains. The supplied values come from the progress code's roughly 67 mm wheels and 108 mm effective track. Verify the IMU's forward-axis sign from the progress configuration and check magnetic interference with the motors powered. Calibrate servo centre and left/right orientation before driving.

Record raw measurements and acceptance results with the commit tested. Add the on-car RAM high-water and timing measurements for the Week 13 evidence once hardware is available.

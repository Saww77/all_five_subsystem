# Subsystem 5 — test log

**Subsystem:** <name>  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | example: runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | | | manual | 1. … 2. … | |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## SE09 obstacle avoidance

Host checks cover echo classification, coarse-to-fine profile and clearance calculations, left and right bypasses, a bounded left fallback, a second obstacle during a detour, and returning to line following. The dispatcher check uses simulated motor feedback to verify stop-before-scan, STOP during a scan, and pausing and resuming a detour after a missing echo. On-car measurements remain pending.

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_scan.c \
  app_program/subsystem5/s5_profile.c -o /tmp/test_s5_scan
/tmp/test_s5_scan

cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_avoidance.c \
  app_program/subsystem5/s5_avoid.c app_program/vehicle_controller/vehicle_core.c \
  -o /tmp/test_s5_avoidance
/tmp/test_s5_avoidance

cc -std=gnu11 -Wall -Wextra -Werror -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ \
  -DCNF_SMP=1 -DTM_MQTT=1 -isystem include -isystem config \
  testing/subsystem5/test_controller_events.c app_program/subsystem5/s5_avoid.c \
  app_program/vehicle_controller/vehicle_core.c -o /tmp/test_s5_controller
/tmp/test_s5_controller

cc -std=gnu11 -Wall -Wextra -Werror testing/subsystem5/test_arc_motion.c \
  app_program/subsystem2/motion_crawl.c -o /tmp/test_s5_arc
/tmp/test_s5_arc
```

### Stationary sonar bench

```sh
make -C build_make -j8 SMP=1 WIFI=none NET=none MQTT=0 S5_BENCH=1
```

Flash `mtk3pico_smp1_usb_cdc_s5bench.uf2` and open the USB serial console. The bench leaves the drive outputs off and exercises GP12 (servo), GP16 (TRIG), and GP17 (ECHO through the divider). Facing in the direction the car drives, the transmitter is on the left and the receiver is on the right when the sensor is centred. Verify that the 30° command points right, 90° points forward, and 150° points left. If the mounted servo moves the other way, copy `config/s5_servo_calibration.example.h` to `config/s5_servo_calibration.h`, set `S5_SERVO_REVERSED` to 1, and rebuild. Use the bench to confirm the actual bearings before a track run.

Place a flat box about 200 mm ahead. Confirm the forward echo, five coarse bearings, fine edge samples, nearest distance, width estimate and chosen side. Repeat with the left and the right side obstructed in turn. Check that an empty forward sector reports open or returns a measured background beyond the clearance threshold. No rising edge, a stuck echo pin, and malformed pulses remain unavailable.

### Track acceptance

Use the MQTT build and keep STOP available. Start on a straight section, with the box ahead and room on one side. Verify that both wheels stop before the sweep, that the car drives only after a fresh clearance reading, and that the outer IR sensors find the original tape before the dashboard returns to Follow line. Repeat with room on the other side and with both sides blocked. The firmware assumes 60 mm from the outer IR sensors to the drive axle. Measure that distance on the assembled car and update `S5_IR_TO_AXLE_MM` if needed before tuning the return alignment.

Start about 300 mm from a flat box on straight tape. The normal scan starts within 230 mm. With a known 200 mm box centred on the tape, check the USB log for the signed edge offsets and sideways target. Move the same box left and then right to verify that the two edge offsets and chosen route change. The planner assumes a 160 mm outside chassis width including its tyres and adds a 40 mm gap. Measure the assembled car at its widest point and adjust `S5_CHASSIS_WIDTH_MM` to match. Check that the car adds a 25 mm sideways step only when the first lane remains blocked. Use a deeper box to verify the clearance check before return, and confirm that the car holds if the return echo becomes unavailable.

A close box beside the chassis can sit outside the forward-biased side readings. During a right bypass the USB log should show the stopped check looking left at 135° and 150°. During a left bypass it should show right at 45° and 30°. Confirm that the servo returns to 90° before the wheels move. After the first clear reading, verify that the car moves 220 mm straight and stops for a second side check before turning toward the tape. Confirm that the rear of the car is past the far edge of the test box before the car turns inward. The controller can travel farther if it detects the box again and has room within the travel limit. Check the logged pivot command and completed left/right encoder counts against the physical heading.

For departure and return, test a broad open area and then a route with limited turning room. On an open route the log should show two 60° rolling turns on departure and another two on return. Both wheels drive forward at different speeds during each arc. If a main bypass turn does not fit, the car holds. If two stopped side scans are inconclusive, check that fresh forward clearance is required before the single move of up to 40 mm to rescan. Verify that the car stops the inward search after two fresh outer IR readings. After realigning, verify that the centre sensor or one outer sensor sees tape in two fresh samples before normal line following resumes. Blank floor after realignment should leave the car stopped.

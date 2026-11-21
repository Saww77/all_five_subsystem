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

For the diagonal path, start at least 500 mm from a flat box with room on the bypass side. The normal line-following scan starts within 400 mm. Check the USB log for `path=diagonal` and `rolling turn 60 deg`. Verify that both wheels roll forward during each turn and that the wheels realign with the tape after the IR crossing. The configured radius at the axle midpoint is 100 mm and the return geometry assumes the outer IR sensors sit 60 mm in front of it. Use a deeper box or a second target beside the return corridor to check that the car passes farther before turning toward the tape. Remove the return echo during a stopped check and verify it holds. The two angled sonar readings sample the return corridor at those bearings.

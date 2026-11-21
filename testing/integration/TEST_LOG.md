# Integration — test log

The motion recovery regression compiles the production steering and controller logic for the host.

```sh
cc -std=c11 -Wall -Wextra -Werror testing/integration/test_motion_recovery.c \
  app_program/subsystem3/line_steering.c \
  app_program/vehicle_controller/vehicle_core.c -o /tmp/test_motion_recovery
/tmp/test_motion_recovery
```

On 21 November 2026 the recovery regression and 23 dashboard tests passed. The Pico W MQTT, offline and isolated motion autotest images linked with the original Subsystem 2 source. A hardware run is recorded separately when performed.

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | all subsystems start and run together (S1–S5 on): every task beats, timer correct | IF-07 (heartbeats), TIMER | manual | Load the `latest-…` build, calibrate, watch the console | `[safety] watching:` lists all 7 tasks, no HUNG; S4 diag `cycles 12` |
| INT-04 | lifting the car stops the motors (S4 → S2) | IF-04 (`CMD_ESTOP` from S4), IF-13 | manual | Drive, pick the car up | `*** LIFTED`, then `[car] ESTOP`, wheels 0 |
| INT-05 | START begins a new hump run (S2 → S4): hand-moved humps before START don't count | `s4_new_run()`, IF-13 | manual | Calibrate, carry the car to the start, START, drive over the ramp | `[imu] new run: highest hump cleared` after START; highest = the ramp's height |
| INT-06 | humps reach MQTT / the dashboard (S4 → S1) | `s4_report()`, IF-06 | manual | `make NET=lwip MQTT=1`, broker + dashboard, drive over the ramp | hump event and highest peak shown; run summary humps = 1 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-13 | INT-03 | `latest-13-11-26` `6ea11cc` | Lim Jin Hin | PASS | `logs/2026-11-13_INT-03-05_all_subsystems.txt` | Jin's car, straight + black ramp, no MQTT: `[safety] watching:` Motor, Mission, Line, Barcode, Terrain, Obstacle Avoid, Telemetry; no HUNG; TICK cycles 12; ultrasonic measured 31-35 cm to an object in front while idle |
| 2026-11-13 | INT-04 | `latest-13-11-26` `6ea11cc` | Lim Jin Hin | PASS | `logs/2026-11-13_INT-03-05_all_subsystems.txt` | 2 pick-ups: LIFTED then ESTOP 10-20 ms later; BACK ON THE FLOOR after |
| 2026-11-13 | INT-05 | `latest-13-11-26` `6ea11cc` | Lim Jin Hin | PASS | `logs/2026-11-13_INT-03-05_all_subsystems.txt` | before the fix: carrying the car to the start logged 16.4 mm as highest (22:30). After commit `e03167f`: `new run: highest hump cleared` on START, ramp 9.8 mm = highest this run (22:45) |
| 2026-11-13 | INT-06 | `latest-13-11-26` `6ea11cc` | Lim Jin Hin | NOT RUN | `logs/2026-11-13_INT-03-05_all_subsystems.txt` | builds with MQTT; needs the team's wifi_secrets.h and a broker |

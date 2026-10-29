# Subsystem 5 — test log

**Subsystem:** Obstacle Avoid (ultrasonic + servo scan, profiling, avoidance planning)  ·  **Student ID:** 2500177  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | obstacle_q pack/unpack round-trip, all type/plan/state codes | CR1 item 7, IF obstacle_q | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | undefined plan code rejected | IF obstacle_q | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | box right of centre → LEFT, closest 25 cm | Stage 3–4 | auto | `subsystem5_test.c` | PASS; prints width/L/R/sidestep |
| S5-U04 | empty scan → CONTINUE | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U05 | obstacle wider than the scan → REVERSE, then STOP when retries run out | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U06 | sine table sign and range | profiling maths | auto | `subsystem5_test.c` | PASS |
| S5-U07 | obstacle_task reports a valid state | telemetry | auto | `subsystem5_test.c` | PASS |
| S5-U08 | no obstacle_q message dropped | CR1, NFR | auto | `subsystem5_test.c` | PASS |
| S5-U09 | distance accuracy at 3.3 V (A3) | A3, R5 | manual | Flat target at 5, 10, 25, 50, 100 cm; 20 pings each; log via bench task | within ±1 cm, no NO_ECHO |
| S5-U10 | sensor-unplugged fault | FAULT_NO_ECHO | manual | Unplug ECHO while in MONITOR | FAULT message within 5 pings (~0.3 s), no hang |
| S5-U11 | servo angles | Stage 1–2 | manual | Command 15/90/165°; photo against a protractor | ±5° |
| S5-U12 | heartbeat never late during a full scan + bypass | NFR8, R4 | manual | Bench build: place obstacle, read `[S5 bench]` worst gap | worst gap ≤ 120 ms, no `HEARTBEAT MISSED` |
| S5-U13 | scan time budget | Week 10 bypass < 3 s | manual | Probe pin (`OA_PROBE_PIN`) on scope: rising = confirmed, falling = plan | record ms; target ≤ 2 s |
| S5-U14 | width estimate on known boxes | Week 10 width error ≤ 10 % | manual | Boxes 10, 15, 20 cm wide at 25 cm | error ≤ 10 % (grade A) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

Host-side copies of S5-U01…U05 (no hardware): `host/test_obstacle.c`.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

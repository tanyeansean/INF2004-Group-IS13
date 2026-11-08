# Subsystem 2 — test log

**Subsystem:** Producer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | left wheel encoder (GP16/17) counts up when the wheel turns forward; right count stays still | encoders | manual + auto | `make TEST=1`, car lifted, turn the LEFT wheel forward exactly 1 revolution when prompted (10 s) | `[TEST] S2-U03 PASS`, counts per revolution printed |
| S2-U04 | right wheel encoder (GP0/1) counts up when the wheel turns forward; left count stays still | encoders | manual + auto | same as S2-U03, RIGHT wheel | `[TEST] S2-U04 PASS`, counts per revolution printed |
| S2-U05 | no invalid A/B transitions (missed edges) during S2-U03/U04 | encoders | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | encoder counts convert to micrometres: 1 revolution = wheel circumference, both signs | odometry | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | motor_task publishes odometry about every 20 ms; 0 mm/s with the wheels still | odometry | auto | `make TEST=1`, hands off the wheels when prompted | `[TEST] S2-U07 PASS`, samples / window / mm / mm/s printed |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-06 | S2-U03 | uncommitted, before `9328d52` | Tan Yean Sean | FAIL | `logs/2026-11-06_S2-U03-U05_encoder.txt` | run 1: left counted down going forward; fixed with ENC_INVERT_LEFT=1 |
| 2026-11-06 | S2-U04 | uncommitted, before `9328d52` | Tan Yean Sean | FAIL | `logs/2026-11-06_S2-U03-U05_encoder.txt` | run 1: left wheel still moving during the right test; 3 s hands-off pause added |
| 2026-11-06 | S2-U03 | `9328d52` | Tan Yean Sean | PASS | `logs/2026-11-06_S2-U03-U05_encoder.txt` | run 2; left counts-per-revolution line not captured |
| 2026-11-06 | S2-U04 | `9328d52` | Tan Yean Sean | PASS | `logs/2026-11-06_S2-U03-U05_encoder.txt` | run 2: right wheel 2487 counts per revolution (82 um per count) |
| 2026-11-06 | S2-U05 | `9328d52` | Tan Yean Sean | PASS | `logs/2026-11-06_S2-U03-U05_encoder.txt` | 0 invalid transitions on both wheels |
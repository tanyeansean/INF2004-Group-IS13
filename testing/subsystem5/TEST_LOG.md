# Subsystem 5 — test log

**Subsystem:** Obstacle Avoid (ultrasonic + servo scan, profiling, avoidance planning, bypass and line recovery)  ·  **Student ID:** 2500177  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | obstacle_q pack/unpack round-trip, all type/plan/state codes | CR1 item 7, IF-09 | auto | `subsystem5_test.c`; host copy in `test_obstacle.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | undefined plan code rejected | IF-09 | auto | as above | `[TEST] S5-U02 PASS` |
| S5-U03 | box right of centre → LEFT, closest 25 cm | Stage 3–4 | auto | as above | PASS; prints width/L/R/sidestep |
| S5-U04 | empty scan → CONTINUE | Stage 4 | auto | as above | PASS |
| S5-U05 | obstacle wider than the scan → REVERSE, then STOP when retries run out | Stage 4 | auto | as above | PASS |
| S5-U06 | sine table sign and range | profiling maths | auto | `subsystem5_test.c` | PASS |
| S5-U07 | obstacle_task reports a valid state | telemetry | auto | `subsystem5_test.c` | PASS |
| S5-U08 | no obstacle_q message dropped | CR1, NFR | auto | `subsystem5_test.c` | PASS |
| S5-U09 | distance accuracy at 3.3 V (A3) | A3, R5 | manual | Flat target at 5, 10, 25, 50, 100 cm, tape-measured; 20 pings each; read `[S5 dbg] ... dist=` | within ±1 cm, no NO_ECHO |
| S5-U10 | sensor-unplugged fault | FAULT_NO_ECHO | manual | Unplug ECHO while in MONITOR | FAULT message within 5 pings (~0.3 s), no hang |
| S5-U11 | servo angles | Stage 1–2 | manual | Command 65/90/115° (the mount allows only 65–115°); photo against a protractor | ±5° |
| S5-U12 | heartbeat never late during a full scan + bypass | NFR8, R4 | manual | Bench build `S5_STANDALONE=1`: place obstacle, read `[S5 bench]` worst gap | worst gap ≤ 120 ms, no `HEARTBEAT MISSED` |
| S5-U13 | scan time budget | Week 10 bypass < 3 s | manual | Probe pin (`OA_PROBE_PIN`) on scope: rising = confirmed, falling = plan | record ms; target ≤ 2 s |
| S5-U14 | width estimate on known boxes | Week 10 width error ≤ 10 % | manual | Boxes 10, 15, 20 cm wide at the stop distance; read `[obs] plan ... width=` | error ≤ 10 % (grade A) |
| S5-U15 | box with something on the left → RIGHT | Stage 4 | auto | `subsystem5_test.c` | `[TEST] S5-U15 PASS` |
| S5-U16 | GP5 echo edges reach S5 through S3's shared GPIO handler | TEAM.md §3 (one IRQ 13 owner) | manual | Integrated build, car idle, nothing close; read 1 Hz `[S5 dbg]` lines | `edges` rises ~2 per ping (~26–32 per line), `noise=0`, `ping=OK` |
| S5-U17 | confirmed obstacle stops a line-following car | IF-09 OBSTACLE, Stage 1 | manual | START; obstacle on the line ahead | `[obs] OBSTACLE at N cm - stopping`, wheels stop (`pwm=0/0`), N ≤ `OA_STOP_DIST_CM` |
| S5-U18 | obstacle reported while the car is idle moves nothing | IF-09, bench safety | manual | Car idle (not started); put an obstacle in front | `[obs] obstacle at N cm (car idle)`, wheels never turn |
| S5-U19 | both sides blocked on the car → REVERSE up to 2 retries, then STOP + NO_SAFE_ROUTE fault | Stage 4, FAULT_BIT_NO_SAFE_RTE | manual | Obstacle wider than the sweep; START | 2× `plan REVERSE`, then `plan STOP`, `[obs] S5 FAULT: no safe route`, car stops |
| S5-U20 | reverse leg covers its distance | Stage 4 "reverse and reattempt", IF-10 | manual | Trigger a REVERSE plan; tape-measure the travel | travel = `OA_REVERSE_CM` ±1 cm; encoder build: `[obs] move BWD` line `L`/`R` agree with the tape within 3 % |
| S5-U21 | full bypass: around the obstacle and back on the line | Stage 4–5 | manual | Obstacle ~10 cm wide on a straight line; START | `[obs] move ... LINE`, then `[obs] S5 done - line following`, car continues on the line, no collision |
| S5-U22 | edges seen from the stop distance: a ~15 cm obstacle does not read L=0 R=0 | Stage 3 profiling | manual | 15 cm-wide obstacle; read the first `[S5 dbg] scan FINE` and `[obs] plan` after the stop | at least one sweep end reads farther than the obstacle; L or R ≠ 0 |
| S5-U23 | 1 µs TIMER correct after every kind of boot | HC-SR04 timing, commit 648d108 | manual | OA_DEBUG build. Boot 3 ways: right after flashing, reset button, cold power-on (power bank) | each boot: `[S5 dbg] timer check: ... (990–1010 us/ms) OK`, and `dist=` reads a hand at 20 cm within ±2 cm |
| S5-U24 | encoder-ended moves: straight distance | Stage 4, IF-10 | manual | Car on the floor; trigger a bypass; tape-measure each straight leg | `[obs] move FWD Ncm ... DONE`; tape = mean of `L`/`R` within 3 %; `stopL`/`stopR` ≥ ask − 2 mm |
| S5-U25 | encoder-ended moves: 45° turns | Stage 4, IF-10 | manual | As S5-U24; mark heading on the floor before/after each turn | heading change 45° ±10° |
| S5-U26 | stalled wheel / dead encoder aborts the move, no false MOVE_DONE | IF-10, IF-11 ABORT | manual | Car on a stand in CAR_AVOID; unplug one encoder before a leg | within ~700 ms: `[obs] move ... FAIL left stalled - stopped, S5 aborted`; wheels stop; no S5 FAULT line |
| S5-U27 | a narrow, far obstacle that the wide beam echoes across the whole fine window is not read as blocked on both sides | Stage 3–4 | auto (host) | `test_obstacle.c`: fine scan 70–110°, 40 cm at every angle | L ≠ 0 and R ≠ 0, plan LEFT or RIGHT (not REVERSE) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

Host-side copies of S5-U01…U05 (no hardware): `test_obstacle.c` in this folder.

## Results

Add a new row for every run. Never edit or delete old rows.

Deviation from `testing/README.md`: the car runs on 2026-11-01/02 were made on
uncommitted working trees, so the commit column names the last commit plus the
changes described in each log's header.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S5-U16 | `72388c2` + working tree | Joe | PASS | `logs/2026-11-01_car_hand_first_integration.txt` | `edges` 2 → 444 over 18 one-second lines, exactly +26 per line every time, `noise=0` on all 34 lines. Ping count is not logged: +26/s fits 2 edges per ping at ~13 pings/s |
| 2026-11-01 | S5-U18 | `72388c2` + working tree | Joe | PASS | `logs/2026-11-01_car_hand_first_integration.txt` | `[obs] obstacle at 24 cm (car idle)`; Mission Control status stayed `mode=0 pwm=0/0` while S5 ran its full scan/plan cycle |
| 2026-11-01 | S5-U17 | `72388c2` + working tree | Joe | PASS | `logs/2026-11-01_car_hand_first_integration.txt` | hand: `[obs] OBSTACLE at 25 cm - stopping` ~1 s after START; car stopped |
| 2026-11-01 | S5-U17 | `72388c2` + working tree | Joe | PASS | `logs/2026-11-01_car_bottle_scan_lines.txt` | water bottle: `OBSTACLE at 25 cm - stopping` |
| 2026-11-01 | S5-U20 | `72388c2` + working tree | Joe | FAIL | `logs/2026-11-01_car_bottle_scan_lines.txt` | "reversing 10 cm" moved ~3 cm (tester, by eye; scan closest 25 → 27 → 29 cm). Cause: timed open-loop reverse in S2, `OBS_MS_PER_CM 65` never measured, PWM 36 barely moves from rest |
| 2026-11-01 | S5-U21 | `72388c2` + working tree | Joe | FAIL | `logs/2026-11-01_car_bottle_scan_lines.txt` | `bypass took too long`: timed legs drove ~⅓ of their planned distance, the line seek never reached the line. Same cause as S5-U20 |
| 2026-11-02 | S5-U17 | `72388c2` + working tree | Joe | PASS | `logs/2026-11-02_car_S5_driven_bypass_L0R0.txt` | 3 of 3 START attempts stopped at 25 cm |
| 2026-11-02 | S5-U19 | `72388c2` + working tree | Joe | PASS | `logs/2026-11-02_car_S5_driven_bypass_L0R0.txt` | 3 of 3: 2× REVERSE, then `plan STOP`, NO SAFE ROUTE fault, car stopped. Behaviour as designed; why both sides read blocked is S5-U22 |
| 2026-11-02 | S5-U20 | `72388c2` + working tree | Joe | FAIL | `logs/2026-11-02_car_S5_driven_bypass_L0R0.txt` | S5-driven `move BWD 10` moved ~2 cm (closest 25 → 27 → 29). Timed move + 200 ms soft-start ramp starting below stall PWM. Ramp removed; moves now end on wheel encoders (fix not yet committed or tested) |
| 2026-11-02 | S5-U22 | `72388c2` + working tree | Joe | FAIL | `logs/2026-11-02_car_S5_driven_bypass_L0R0.txt` | from 25 cm every angle 65–115° read the obstacle (e.g. `65:25 … 115:26`): no edge visible, L=0 R=0. Fix: `OA_STOP_DIST_CM` 25 → 35 (not yet committed or tested) |
| before 2026-11-17 | S5-U23 | not recorded (build before `302a8e2`) | Joe | FAIL | tester report, no log saved | sensor right after flashing, then out of range after every reset button or power-bank/PC power-on. Matches the 43×-slow TIMER found in `648d108`; S5's own copy fixed in `s5_hw.c` (not yet committed or tested) |
| 2026-11-17 | S5-U01 | `302a8e2` (+ working tree; host modules unchanged) | Joe | PASS | `logs/2026-11-17_host_logic.txt` | host build: header round-trip, 160 combinations; byte fields 0..255 |
| 2026-11-17 | S5-U02 | `302a8e2` (+ working tree; host modules unchanged) | Joe | PASS | `logs/2026-11-17_host_logic.txt` | host build: plan codes 5..7 rejected |
| 2026-11-17 | S5-U03 | `302a8e2` (+ working tree; host modules unchanged) | Joe | FAIL | `logs/2026-11-17_host_logic.txt` | host build: got REVERSE (4), L=0. **Test is stale, not the code:** it builds a 40–140° scan, but the sweep is now 65–115° (`OA_MAX_SCAN_POINTS` = 11), so the scan is cut at 90° and the box touches its end. Fix the test's angles; the on-board S5-U03 uses the real sweep and has not been run |
| 2026-11-17 | S5-U04 | `302a8e2` (+ working tree; host modules unchanged) | Joe | PASS | `logs/2026-11-17_host_logic.txt` | host build |
| 2026-11-17 | S5-U05 | `302a8e2` (+ working tree; host modules unchanged) | Joe | PASS | `logs/2026-11-17_host_logic.txt` | host build: wall → REVERSE, retries out → STOP |
| 2026-11-17 | S5-U27 | `302a8e2` (+ working tree, before the step 6 fix) | Joe | FAIL | `logs/2026-11-17_host_logic_step6_fix.txt` (control section) | L=0 R=0 → REVERSE. Bug: `profile_build()` step 6 treated an end of the *fine window* as an unknown edge, though the coarse scan saw nothing beyond it. Explains reverses with the obstacle still far away |
| 2026-11-17 | S5-U27 | `302a8e2` (+ working tree, step 6 fix) | Joe | PASS | `logs/2026-11-17_host_logic_step6_fix.txt` | step 6 now only treats the *servo sweep* ends (65°/115°) as unknown |
| 2026-11-17 | S5-U03 | `302a8e2` (+ working tree, step 6 fix, host test on 65–115°) | Joe | PASS | `logs/2026-11-17_host_logic_step6_fix.txt` | host test moved to the real sweep; the earlier FAIL was the stale 40–140° scan |
| 2026-11-17 | S5-U01, U02, U04, U05 | `302a8e2` (+ working tree, step 6 fix) | Joe | PASS | `logs/2026-11-17_host_logic_step6_fix.txt` | host build, regression after the fix: 0 failures |

## Not yet run

No result exists for these yet. Run each on the next flashed build and add a row above.

| Test ID | What to do |
|---|---|
| S5-U01…U08, U15 (on board) | `cd build_make && make NET=lwip MQTT=1 TEST=1 -j8`, flash, save the console to `logs/` |
| S5-U23 | flash the latest `.uf2`, then reset and power-cycle; check the `timer check` line each time |
| S5-U22, U20, U24, U25, U26, U21 | on the car, latest build, in that order (U21 depends on the others) |
| S5-U09, U10, U11, U12, U13, U14 | manual bench tests, steps in the table above |

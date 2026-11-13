# Subsystem 4 — test log

**Subsystem:** IMU / terrain (GY-511): humps, hump height, lifted or overturned, moving / stationary / turning (with the wheel encoders)  ·  **Student ID:** 2500928  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

Requirement change, 13 Nov 2026: collision detection with the IMU is removed. The IMU must instead detect the car being lifted or overturned, and the motors must stop at once. Lifting the car ends the run at the demonstration.

## Test cases

Define a test here before running it. Test IDs never change.

S4-U01 and S4-U02 were the template's examples and are retired (IDs not reused).

### Laptop tests of the detector logic (no car)

Run with `sh testing/subsystem4/pc/run.sh`. They build `terrain_logic.c` exactly as the car uses it and feed it made-up readings every 10 ms. They show the logic is right. They do not show that the limits suit the real car: the lift and overturn readings are made up from physics, not recorded.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-P01 | still for 5 s is not "lifted" | lifted / overturned | auto (PC) | `pc/terrain_sim.c` | `[TEST] S4-P01 PASS` |
| S4-P02 | 30 s of driving with SLOW-sized shaking is not "lifted" | lifted / overturned | auto (PC) | same | PASS |
| S4-P03 | 30 s of driving with FAST-sized shaking is not "lifted" | lifted / overturned | auto (PC) | same | PASS |
| S4-P04 | the ramp (7° over 1 s) is counted as a hump of 9–12 mm and is not "lifted" | hump height; lifted / overturned | auto (PC) | same | PASS |
| S4-P05 | the ramp taken fast (7° over 0.4 s, FAST shaking) is not "lifted" | lifted / overturned | auto (PC) | same | PASS |
| S4-P06 | one hard bump from below and one wild reading are not "lifted" | lifted / overturned | auto (PC) | same | PASS |
| S4-P07 | 12° of pitch for 0.3 s (hard braking) is not "lifted" | lifted / overturned | auto (PC) | same | PASS |
| S4-P08 | lifted level 20 cm in 0.6 s while driving gives LIFTED within 300 ms | lifted / overturned | auto (PC) | same | PASS, time printed |
| S4-P09 | lifted slowly and level (20 cm in 1 s), then tipped 35°, gives LIFTED once tipped. The slow level lift alone is a known limit; the test prints whether it was detected | lifted / overturned | auto (PC) | same | PASS |
| S4-P10 | tipped to 35° and held gives LIFTED within 600 ms | lifted / overturned | auto (PC) | same | PASS, time printed |
| S4-P11 | rolled onto its side gives OVERTURNED, off the floor within 500 ms | lifted / overturned | auto (PC) | same | PASS, time printed |
| S4-P12 | flipped upside down gives OVERTURNED | lifted / overturned | auto (PC) | same | PASS |
| S4-P13 | tilting the car about while it is carried counts no hump and leaves the highest height unchanged | hump height | auto (PC) | same | PASS |
| S4-P14 | held level in a shaky hand for 2.5 s stays "off the floor" | lifted / overturned | auto (PC) | same | PASS |
| S4-P15 | put down level and still gives PUT_DOWN within 2.5 s | lifted / overturned | auto (PC) | same | PASS |
| S4-P16 | after being put down, the ramp is counted as a hump again | hump height | auto (PC) | same | PASS |
| S4-P17 | wheels stopped and no shake is stationary | motion events; wheel encoders | auto (PC) | same | PASS |
| S4-P18 | wheels turning on a smooth floor (no shake) gives MOVING within 100 ms | motion events; wheel encoders | auto (PC) | same | PASS |
| S4-P19 | wheels stop and the shake dies down gives STATIONARY | motion events; wheel encoders | auto (PC) | same | PASS |
| S4-P20 | encoders read 0 (unplugged) but the car shakes: still MOVING, never TURNING | motion events; wheel encoders | auto (PC) | same | PASS |
| S4-P21 | both wheels at 150 mm/s is not turning | turn rate | auto (PC) | same | PASS |
| S4-P22 | left 60, right 240 mm/s gives TURNING left at about 94°/s (wheel spacing 110 mm, measured 13 Nov) | turn rate | auto (PC) | same | PASS, rate printed |
| S4-P23 | wheels equal again gives TURN ENDED | turn rate | auto (PC) | same | PASS |
| S4-P24 | a 100 ms steering correction is not counted as a turn | turn rate | auto (PC) | same | PASS |
| S4-P25 | wheels spinning unevenly while the car is lifted is not counted as a turn | turn rate; lifted / overturned | auto (PC) | same | PASS |
| S4-P26 | one encoder that never moves is not counted as a turn | turn rate | auto (PC) | same | PASS |
| S4-P27 | put down and driven off at once: back "on the floor" within 3.5 s and the next hump is counted | lifted / overturned; hump height | auto (PC) | same | PASS |

### On the car

Save the console output of each run in `logs/`. The `[imu]` status line prints, once a second, the biggest `lean` (degrees) and `up` (m/s) of that second: these show how close the run came to the limits (lean 25°, up 0.35 m/s).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-L01 | normal driving never reads as lifted | lifted / overturned | manual | Build with `S4_LIFT_STOPS` 0 (report only). Drive at least 5 laps of the track with the ramp at the normal speed. | no `LIFTED` or `OVERTURNED` line; note the biggest `lean` and `up` seen |
| S4-L02 | a normal pick-up stops the motors | lifted / overturned; IF-04 | manual, video | Normal build. While the car follows the line, pick it up the way a person naturally would. Repeat 5 times. | each time: `*** LIFTED`, then `[car] ESTOP`, wheels stop; `stops` goes up by 1 |
| S4-L03 | a slow, level lift | lifted / overturned | manual | As S4-L02, but lift slowly and keep the car level; then tip it. | record whether the level lift alone was detected, and the `up` value; LIFTED at the latest when tipped |
| S4-L04 | overturned stops the motors | lifted / overturned; IF-04 | manual, video | While the car drives, roll it onto its side; repeat, turning it upside down. | `*** LIFTED` and/or `*** OVERTURNED`, `[car] ESTOP`, wheels stop |
| S4-L05 | the car can start again after a lift | lifted / overturned | manual | After S4-L02, put the car on the line, hands off for 2 s, press start, drive over the ramp. | `BACK ON THE FLOOR`; the car starts; `HUMP DONE` with a height near 10 mm |
| S4-L06 | carrying the car adds no hump | hump height | manual | Note "highest this run". Pick the car up, tilt it about, put it down. | no `HUMP DONE` while carried; the highest height is unchanged |
| S4-L07 | time from lift to wheels stopped | lifted / overturned | manual, video | Film S4-L02 from the side, slow motion if possible. | time recorded |
| S4-L08 | the ramp at the fastest speed the team will use is not "lifted" | lifted / overturned | manual | Report-only build, fastest planned speed, 5 ramp crossings. | no `LIFTED`; biggest `lean` and `up` noted |
| S4-L09 | the wheel encoders reach subsystem 4 | wheel encoders (`s2_odom_get`) | manual | Car lifted, turn each wheel forward by hand, then drive a straight. Watch `wheels` on the `[imu]` status line. | `wheels ok`, L and R both positive going forward and close to each other on a straight; `MOVING` when the wheels turn |
| S4-L10 | turn rate from the wheels against a known turn | turn rate | manual | `TRACK_MM` = 110 mm (measured 13 Nov). Turn the car 90° on the spot (or drive a marked curve) and time it. | `TURNING LEFT` / `RIGHT` in the right direction; the rate times the time is close to 90° |
| S4-L11 | line following on a straight is not reported as turning | turn rate | manual | Drive a straight section 5 times. | no `TURNING` line, or note how often it appears (the limits then need raising) |
| S4-L12 | hump length cross-check | hump height; wheel encoders | manual | Drive over the ramp 3 times. | after each `HUMP DONE`, `right wheel rolled N mm`; N noted against the ramp's 198 mm |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (PC)** = the laptop program prints `[TEST] <ID> PASS|FAIL`;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-P01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-13 | S4-P01..P27 | uncommitted, `buddy4-lift` on `5f55eb4` | Lim Jin Hin | PASS | `sh testing/subsystem4/pc/run.sh` | 27 passed, 0 failed |
| 2026-11-13 | S4-L01 | uncommitted, `buddy4-lift` on `5f55eb4` | Lim Jin Hin | PARTIAL | `logs/2026-11-13_S4-L01_power36_straights.txt` | straight track + black ramp only (no curves). Report-only build, team power 36: car stalled 2 of 3 runs (once on the ramp, once on flat); 1 good run: hump 9.9 mm, no false lift (up max 0.01 m/s, lean max 6.8 deg) |
| 2026-11-13 | S4-L01 | uncommitted, `buddy4-lift` on `5f55eb4` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L01-L02_power45_straights.txt` | straight + ramp only, TEST-ONLY power 45 (Sean's setting not changed in the branch): 5 of 5 runs over the ramp, humps 9.9, 10.4, 10.3, 10.2, 10.3 mm; no false lift (up max 0.02 m/s, lean max 7.3 deg). Found: old up-speed limit 0.35 m/s missed 3 of 6 hand pick-ups; limit lowered to 0.06 m/s after this run |
| 2026-11-13 | S4-L02 | uncommitted, `buddy4-lift` on `5f55eb4` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L01-L02_power45_straights.txt` | limit 0.06 m/s, power 45: 5 of 5 pick-ups while driving gave LIFTED then `[car] ESTOP` 5-22 ms later, wheel speed 0 on the next status line; 2 before the ramp, 1 on the ramp (lean 7.2 deg), 2 after. No false lift while driving. 2 humps in these runs: 10.0, 9.7 mm |
| 2026-11-13 | S4-L04 | uncommitted, `buddy4-lift` on `5f55eb4` | Lim Jin Hin | PARTIAL | `logs/2026-11-13_S4-L01_power36_straights.txt` | by hand, car not driving: on its side gave OVERTURNED at 60.5 deg. Not yet done while driving, upside down not tried |
| 2026-11-13 | S4-L05 | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | after 4 lifts while driving (each stopped the wheels), put down, restarted with GP20: next humps 9.1 and 10.1 mm, junction stops. Found: holding the car 5-9 deg off level for 3 s counted as put down; rule tightened to 4 deg (S4-P28) |
| 2026-11-13 | S4-L06 | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | carried and tilted about with the car parked: LIFTED at once, OVERTURNED at 62 deg while tilted, no hump counted, BACK ON THE FLOOR after |
| 2026-11-13 | S4-L04 | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | tipped sideways by hand while driving, twice: stop 0.7-0.9 s after the tip began (lean rule, 25 deg). Air wheel spun 245 mm/s, floor wheel 0, so a spin rule was added after this test (S4-P29, not yet car-tested) |
| 2026-11-13 | S4-L03 | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | slow level lift (about 20 cm in 2 s), twice: LIFTED at lean 0.4 and 2.7 deg, up speed 0.06-0.07 m/s, before any tipping |
| 2026-11-13 | S4-L10 | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | PARTIAL | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | quarter turns by hand on the floor, motors off: rate from the wheels 17-31 deg/s, summed over the turn about 87 deg each way (worked out from the 1 s samples; real 90). Below the 45 deg/s TURNING limit, so no event; a 90-deg barcode turn (450 ms) is about 200 deg/s. Not yet checked on a real turn |
| 2026-11-13 | S4-L07 | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | PASS | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | phone video of start -> lift -> wheels stop, kept by Jin (not in the repo) |
| 2026-11-13 | (diagnostic) | uncommitted, `buddy4-lift` on `0c99250` | Lim Jin Hin | FOUND | `logs/2026-11-13_S4-L03-L10_lift_tip_turn_timer.txt` | 1 us TIMER ran 23,437 us/s with watchdog TICK cycles 0; test-only build with drv_track.c writing the tick unconditionally: 999,755 us/s (after a load) and 999,920 us/s (cold start). See PROJECT_CONTEXT.md v2 |

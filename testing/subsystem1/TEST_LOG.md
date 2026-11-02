# Subsystem 1 — test log

**Subsystem:** Telemetry (WiFi / MQTT) + Safety Monitor (Buddy 1, Aurelius Ng Ker Fong)  ·  **Student ID:** 2500612  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results. Overview of the subsystem: `app_program/subsystem1/README.md`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | 7-bit signed field packs and unpacks on the target compiler | CR1 | auto | `make NET=lwip MQTT=1 TEST=1`, flash, read console | `[TEST] S1-U01 PASS` |
| S1-U02 | Telemetry task sets its heartbeat bit | G2, flg_hb | auto | as S1-U01 | `[TEST] S1-U02 PASS` |
| S1-U03 | A task that never beat is not reported hung | Safety Monitor | auto | as S1-U01 | `[TEST] S1-U03 PASS` |
| S1-U04 | No beat for exactly the limit is still on time | Safety Monitor | auto | as S1-U01 | `[TEST] S1-U04 PASS` |
| S1-U05 | Past the limit = hung; Obstacle Avoid has a longer limit | Safety Monitor | auto | as S1-U01 | `[TEST] S1-U05 PASS` |
| S1-U06 | A beat clears hung and reports the recovery | Safety Monitor | auto | as S1-U01 | `[TEST] S1-U06 PASS` |
| S1-U07 | Hung task number for the telemetry fault mask | Safety Monitor, live data | auto | as S1-U01 | `[TEST] S1-U07 PASS` |
| S1-P01 | Safety Monitor logic on a simulated 10 ms tick: 10 min healthy with no false trip; a frozen task reported within its limit + 2 checks | Safety Monitor | auto (PC) | `sh testing/subsystem1/pc/run_pc_tests.sh` | `ALL PASSED` |
| S1-P02 | MQTT reconnect is paced: no "already connecting" spam, one retry timer | mqtt_utk.c | auto (PC) | as S1-P01 | `ALL OK` twice |
| S1-M01 | Power bank only, 2 min, car idle | NFR5 | manual | 1. Flash. 2. Unplug USB, power bank only. 3. Laptop monitor. | `[status] online` stays; live data keeps coming; `lost=0` |
| S1-M02 | WiFi drop and recovery | NFR5 | manual | Hotspot off 30 s, then on | Reconnects by itself; `[faults] ... WIFI_LOSS` while down, then clears 5 s after |
| S1-M03 | Hung task stops the car | NFR8, IF-04, flg_faults | manual | Owner adds a temporary `tk_slp_tsk(TMO_FEVR);` in their task loop after READY; start the car | Console `[safety] HUNG (no heartbeat) - E-stop: <task>`; car stops; laptop `[faults] TASK_HUNG, hung task: <task>` |
| S1-M04 | Hung Mission or Motor Control resets the chip | NFR8, watchdog | manual | As S1-M03 in Mission Control or Motor Control | Chip resets within ~100 ms of the report; next boot prints `[safety] last reset: watchdog, <task> was hung`; laptop shows the task for 10 s |
| S1-M05 | WiFi signal strength in the heartbeat | heartbeat | manual | Laptop monitor while online | `rssi=-NNdBm` on the live line (not 0) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (PC)** = runs on a laptop with gcc, no Pico;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

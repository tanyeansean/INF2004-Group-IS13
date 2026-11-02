# Subsystem 1: Telemetry + Safety Monitor (Buddy 1, Aurelius Ng Ker Fong)

Two parts, one folder:

1. **Telemetry**: the car's packed MQTT data, commands, barcode map and the end-of-run summary (robocar_comms v0.4, Change Requirement 1). The network itself runs in the kernel's WiFi task; this folder hands it bytes through two message buffers.
2. **Safety Monitor**: every task sets its heartbeat bit; the Safety Monitor checks them every 20 ms, E-stops the car if a task hangs, and runs the hardware watchdog.

## Files

| File | What it does |
|---|---|
| `subsystem1.c / .h` | Kernel objects (`flg_hb`, `mbf_status`, MQTT buffers), starts both tasks, IF-05 helpers `s1_post_*()` |
| `task_telemetry.c` | Telemetry task (priority 16, core 0): status_q → packed snapshot, `comms_poll()`, `telem_poll()`, fault mask |
| `telemetry.c / .h`, `packets.c / .h` | Packed live data, events, heartbeat (unchanged from robocar_comms v0.3.1) |
| `comms_utk.c`, `comms.h`, `comms_config.h` | Link state, 1-byte commands and acks, heartbeat, barcode map |
| `s1_mqtt_bridge.c` | Runs in the kernel's WiFi task: moves bytes between it and the Telemetry task |
| `hb_bits.h` | Heartbeat bit of each task |
| `task_safety.c` | Safety Monitor task (priority 10, core 1): attendance, E-stop, watchdog |
| `safety_logic.c / .h` | Attendance logic and hung limits, no kernel calls (also runs in the PC test) |
| `subsystem1_test.c` | On-target tests S1-U01 to S1-U07 (`make TEST=1`) |

Shared kernel files carrying the network for this subsystem (agreed with the team, TEAM.md section 3):
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c / .h`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`.
Laptop tools: `integrated_car/tools/` (`telemetry_monitor.py`, `packets.py`, `barcode_map.py`, `mosquitto.conf`).

## Changes in this version (2 Nov 2026)

**Telemetry**
- **WiFi power saving off** (`cyw43_utk.c`). With power saving on, the car draws so little current that many USB power banks switch themselves off, and commands can wait up to 200 ms.
- **Each board has its own MQTT client ID** (`mqtt_utk.c`): `MQTT_CLIENT_ID` + the last 3 bytes of the board's MAC, e.g. `mtk3smp-pico-0ab23f`. Two boards with the same ID kick each other off the broker. The console prints the ID when it connects.
- **Paced reconnect** (`mqtt_utk.c`): a connection that had been working reconnects at once; a failed attempt waits 3 s. Before, an unreachable broker made it retry on every 2 ms poll and print "already connecting" each time.
- **WiFi signal strength** in the heartbeat again (`cyw43_utk.c` reads it once a second; it was always 0 in the uT-Kernel port).
- **Fault mask in live data now filled in**, so the laptop can show why the car stopped without the USB console:

  | Bits | Meaning |
  |---|---|
  | 0-11 | The team's fault table (`flg_faults`, `subsystem2/shared_types.h`), read without changing it: e.g. bit 8 `US_NO_ECHO`, bit 2 `COLLISION`, bit 6 `TASK_HUNG` |
  | 12-14 | Task the Safety Monitor found hung: 1 Motor, 2 Mission, 3 Line, 4 Barcode, 5 Terrain, 6 Obstacle, 7 Telemetry, 0 none |
  | 15 | WiFi loss (report only) |

  The monitor prints a line whenever it changes, e.g. `[faults] 0x0100: US_NO_ECHO`.

- **Run summary (`run/summary`, new):** at the end of every run (state back to STOPPED, ERROR or IDLE) the car publishes one 16-byte retained message with everything the run produced, consolidated from the same raw data it sent as live data and events: duration, distance, top speed, barcodes read and their letters, junction moves, humps and the highest peak (mission requirement 5), obstacles and the closest one, every fault seen, reconnects and dropped events. Type `summary` for the totals so far. The monitor tallies the raw events itself and prints `[run summary] matches the raw data seen by this monitor`, or the fields that differ. Layout: `packets.h` (`PKT_SUM_*`). Nothing to do in the other subsystems: it is built from the data they already send.

**Safety Monitor (new)**: see below.

## Safety Monitor

Every 20 ms (20-30 ms with the 10 ms kernel tick) `safety_task`:
1. Reads and clears all heartbeat bits in one call (`tk_wai_flg(flg_hb, HB_ALL_TASKS, TWF_ORW | TWF_BITCLR, &seen, TMO_POL)`), so a beat that lands during the check is never lost.
2. A task with no beat for longer than its limit is **hung**. A task is only watched after its first beat, so a subsystem that is not built in or not started is never reported.
3. Hung task other than Telemetry:
   - sets `FAULT_BIT_TASK_HUNG` on `flg_faults` (bit 6 in Sean's table, latched);
   - sends `CMD_ESTOP` to Mission Control on `mbf_cmd` (IF-04, the same path as the laptop's `estop`), again every 200 ms while it stays hung, so a START cannot drive on.
4. If Mission Control or Motor Control is the hung one, the E-stop cannot get through them: it **stops kicking the watchdog**, the chip resets within 100 ms and the motor pins go low (brake). The task's number goes into watchdog scratch register 0; the next boot prints `[safety] last reset: watchdog, <task> was hung` and the laptop shows the task for 10 s.
5. Otherwise it kicks the watchdog.

Telemetry late = report only: WiFi trouble must never stop the car (NFR5).

**Start-up:** monitoring and the watchdog start once line calibration is done (`s3_is_calibrated()`), so the 10 s console waits and the calibration countdown at boot never count.

**Limits** (`safety_logic.h`, PROVISIONAL until measured on the car):

| Task | Wakes every | Limit |
|---|---|---|
| Motor Control | 20 ms timeout | 60 ms |
| Mission Control | each line message, 20 ms timeout | 60 ms |
| Line Follow | 5 ms | 60 ms |
| Barcode Decode | 5 ms | 60 ms |
| Terrain Monitor | about 20 ms | 60 ms |
| Obstacle Avoid | 50 ms chunks + 30 ms ping | 150 ms |
| Telemetry | 10 ms | report only |

The G9 table said 40 ms, but that assumed a 1 ms tick. Here the tick is 10 ms (`config/config.h`, `CNF_TIMER_PERIOD`): a "20 ms" wait really takes 20-30 ms. In the PC simulation, a frozen task is reported 25-95 ms after it freezes (limit + up to two checks); the E-stop then takes one Mission Control loop (≤ 30 ms).

**Switches** (top of `task_safety.c`):
- `SM_ENFORCE 0`: report only (console, fault bits, laptop), no E-stop and no reset. Use while tuning the limits.
- `SM_WATCHDOG 0`: no hardware watchdog.

**Console lines:** `[safety] watching: ...` (once, after calibration), `[safety] hardware watchdog on, 100 ms`, `[safety] HUNG (no heartbeat) - E-stop: <task>`, `[safety] heartbeat back: <task>`. Mission Control then prints `[car] ESTOP (MQTT)`, because the stop arrives on the same queue as the laptop's.

## Build and flash

1. `config/wifi_secrets.h` (gitignored, copy `config/wifi_secrets.example.h`): hotspot name and password, the laptop's IP. `MQTT_CLIENT_ID` can stay as it is: the MAC suffix keeps it unique.
2. `cd build_make && make NET=lwip MQTT=1` → `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.
3. On-target tests: `make NET=lwip MQTT=1 TEST=1`, then look for `[TEST] S1-` on the console.

Calibration still follows the console prompts at boot (Buddy 3's line task). On the power bank alone you cannot see them, so calibrate with the PC attached, or follow the timing: about 10 s after power-on, a 3 s countdown, then hold both front sensors on black; another 3 s countdown, then hold them on white. The console says each hold is 1.5 s, but with the 10 ms tick it can take up to 3 s, so hold each position for 3 s. If the car sits still on one surface for both, calibration still "passes" with meaningless thresholds (it only prints a warning), and the car then stops at random "junctions".

## Laptop

```
mosquitto -c integrated_car/tools/mosquitto.conf -v
pip install "paho-mqtt==2.1.0"
python integrated_car/tools/telemetry_monitor.py
```
Why the car stopped, without the USB console:
- `[event/state] LINE_FOLLOW -> OBSTACLE_SCAN`: the ultrasonic saw something within 35 cm.
- `-> ERROR` with `[faults] ... US_NO_ECHO`: the ultrasonic is not answering.
- `-> STOPPED` with `line=011`: both front sensors read black (junction stop).
- `[faults] TASK_HUNG, hung task: <task>`: the Safety Monitor stopped it.

## Tests

| Where | How | Result (2 Nov 2026) |
|---|---|---|
| PC: Safety Monitor logic, MQTT reconnect | `sh testing/subsystem1/pc/run_pc_tests.sh` (repo root, gcc) | ALL PASSED / ALL OK |
| PC: packed layouts and the run summary (robocar_comms v0.4) | `python test/run_tests.py` in robocar_comms | 509 checks, ALL PASSED; end-to-end through Mosquitto: summary matches the raw events |
| Car: S1-U01 to S1-U07 | `make NET=lwip MQTT=1 TEST=1` | not run yet |
| Car: S1-M01 to S1-M05 | `testing/subsystem1/TEST_LOG.md` | not run yet |

Builds checked: `NET=lwip MQTT=1`, `NET=lwip MQTT=1 TEST=1`, `NET=lwip MQTT=0`, `NET=none` with no warnings (Pico SDK 2.3.1, GCC 15.2); the Safety Monitor files also compile clean with `-Wall -Wextra` in every switch combination.

## Needs team agreement

- **TEAM.md section 4:** a row for `safety_task – 10 – core 1`. Priority 10 is outside S1's band (16-17) on purpose: the Safety Monitor must outrank the tasks it checks. 5-11 is unused.
- **TEAM.md section 5:** Safety Monitor → Mission Control uses the existing IF-04 (`mbf_cmd`, `CMD_ESTOP`) and `FAULT_BIT_TASK_HUNG` on `flg_faults` (already in `shared_types.h`).
- **Shared kernel files** above (`cyw43_utk.c / .h`, `mqtt_utk.c`): Yong Quan made the port; let him check the change.
- **Hang test (S1-M03, S1-M04):** each owner adds a temporary `tk_slp_tsk(TMO_FEVR);` in their own task to show the stop, then removes it.

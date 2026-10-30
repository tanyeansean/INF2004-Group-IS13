# HANDOFF — Buddy 3 (Track Purist), Group IS13

Read this first next session to restore full context. Written 2026-10-22.

---

## 1. Who / what this is

- Course: INF2004 Embedded Systems, Group **IS13**, Tribunal 1.
- I am **Buddy 3 = "Track Purist"** (Student 2500377, Wong Yong Quan).
- My subsystem = **IR sensors**: line following + barcode reading.
- Hardware: **Raspberry Pi Pico W + Robo Pico** board.
- Language/SDK: **C with the Pico C SDK** (SDK 2.3.1 via VS Code Pico extension).
- RTOS (final target): **micro T-Kernel (uT-Kernel 3.0)** — NOT yet integrated (project risk A7).
- Repo: https://github.com/tanyeansean/INF2004-Group-IS13
  - Cloned locally at: `D:\SIT\Y2T1\embeddedproj\INF2004-Group-IS13`
  - My work is on local branch **`buddy3-track-purist`** — **NOT pushed** to GitHub.

## 2. THE SENSOR LAYOUT (important — doc is wrong, this is correct)

**3 IR sensors** (the tribunal doc says 4 in places — ignore that, it's stale):

| Sensor | Job | Signal | Pin (INTENDED, unverified) |
|--------|-----|--------|-----------------------------|
| **Left**   | line following | analog | GP26 / ADC0 (phys pin 31) |
| **Center** | line following | analog | GP28 / ADC2 (phys pin 34) |
| **Right**  | barcode (right side only) | digital, edge interrupt | GP6 (phys pin 9) — **PLACEHOLDER, never confirmed** |

Line following = difference of Left vs Center ADC. Barcode = Right sensor
edge timestamps decoded as Code 39. Barcodes are on the RIGHT only (A9).

## 3. What is BUILT and VERIFIED

Everything lives under `track_purist/`. Module is RTOS-agnostic: pure
logic is host-testable; only `rtos_port_utk.c` touches the RTOS.

```
track_purist/
  include/  track_types.h  line_follow.h  barcode_decode.h  ir_calibrate.h  rtos_port.h
  src/      line_follow.c  barcode_decode.c  ir_calibrate.c  rtos_port_utk.c (STUB)
  test/     test_barcode.c  test_line_follow.c  test_util.h  CMakeLists.txt
  selftest/ track_purist_selftest.c  CMakeLists.txt  pico_sdk_import.cmake
  CMakeLists.txt  README.md  .gitignore
```

**Host unit tests PASS: 112 barcode + 33 line-follow checks, 0 failed.**
Clean build, zero warnings. Tested logic:
- Line: diff-based error (signed 10-bit), junction (both-black held >= N ms,
  held HIGH while both black), lost-line, calibration threshold math.
- Barcode: 15 bars = 30 edges = 29 widths Code 39 frame `* <cmd> *`,
  3-wide-of-9 classification, both `*` guards, reversed-frame retry,
  Z -> INVALID, 32-entry edge ring w/ overflow-as-counter, pure frame
  assembler (edge-arrived / time-passed) proven: 1 barcode -> 1 VALID etc.

**Bare-metal self-test BUILDS into a .uf2** (verified on this machine):
`track_purist/selftest/build/track_purist_selftest.uf2` (~68 KB).

## 4. Commands (copy-paste)

### Run host unit tests (laptop only, no Pi):
```powershell
cd D:\SIT\Y2T1\embeddedproj\INF2004-Group-IS13\track_purist\test
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
ctest --test-dir build --output-on-failure
```
(Uses MinGW gcc 15.2.0 + cmake 4.1.0 on PATH.)

### Build the Pi self-test .uf2 (command line):
```powershell
$env:PICO_SDK_PATH="$env:USERPROFILE\.pico-sdk\sdk\2.3.1"
$env:PICO_TOOLCHAIN_PATH="$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1"
$env:Path="$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin;$env:USERPROFILE\.pico-sdk\cmake\v4.3.4\bin;$env:USERPROFILE\.pico-sdk\ninja\v1.13.2;$env:Path"
cd D:\SIT\Y2T1\embeddedproj\INF2004-Group-IS13\track_purist\selftest
cmake -S . -B build -G Ninja
cmake --build build
# output: build\track_purist_selftest.uf2
```
(Easier alternative: open the `selftest` folder in VS Code with the
Raspberry Pi Pico extension, click **Compile**.)

### Flash + watch:
1. Hold **BOOTSEL** on the Pico, plug in USB -> it mounts as `RPI-RP2`.
2. Drag `track_purist_selftest.uf2` onto it. It reboots.
3. Open serial monitor (VS Code serial icon, or PuTTY) at **115200 baud**.

## 5. How the self-test behaves

On boot (3 s delay for USB), then:
1. Prompts: hold Left+Center over **BLACK LINE** ~1.5 s, then **BACKGROUND** ~1.5 s (calibration / A2).
2. Prints `L=#### C=#### | err=#### junction=# lost=#` ~5x/sec.
3. Prints `>>> BARCODE: VALID cmd=LEFT/RIGHT/STRAIGHT/U_TURN` or `INVALID` when the Right sensor sees a barcode.

Expected: err swings +/- and is ~0 centered; junction=1 when both on black;
lost=1 off-track; sample barcode -> VALID; garbage -> INVALID.

## 6. OPEN QUESTIONS — must resolve on the day (I will ask these)

1. **Right-IR pin** — code uses GP6 as a PLACEHOLDER. What is the real pin?
2. **Wiring type** — Grove connectors or Dupont jumpers? Robo Pico Grove
   ports map to FIXED GP pins, so if Grove, the port decides the pin
   (not free choice). NEED photo of Robo Pico board + IR sensor/cable,
   OR exact model numbers.
3. **Sensor voltage** — TCRT5000 modules are often 5V; Pico ADC is 3.3V
   max. Confirm module runs at 3V3 so analog out stays <= 3.3V (A2).
4. **GP26 / VBAT jumper** — doc flags a Robo Pico caution; confirm GP26
   is free.
5. **Barcode command mapping** — placeholder A/B/C/D -> LEFT/RIGHT/
   STRAIGHT/U_TURN. Confirm the real physical barcode-to-command meaning.
6. **`line_err_q` exact bit layout** must be agreed with Buddy 2 (Sean)
   so his Mission Control unpacks exactly what I pack.
7. **Fault bits** — deferred to Sean's shared fault-mask table (gap G1).
   I define NONE; add only the bit(s) he assigns.

## 7. Known limitations / what is NOT done

- `rtos_port_utk.c` is a **STUB**: rtos_delay_ms does nothing,
  rtos_now_ms returns 0, rtos_queue_send always "succeeds",
  rtos_flag_set is a no-op. The self-test deliberately bypasses all this
  (uses plain sleep_ms + printf), so it proves SENSING + LOGIC on
  hardware, NOT the RTOS integration.
- No `main()` / task creation for the real RTOS build (that overlaps
  Sean's Mission Control skeleton).
- uT-Kernel itself is not installed anywhere and not integrated with the
  Pico SDK (team-level risk A7).
- Pin numbers (GP26/28/6) are the DOC'S INTENDED plan, unverified vs the
  physical board.

## 8. Git state

- Branch `buddy3-track-purist`, ~12+ commits ahead of `main`, NOT pushed.
- `selftest/` folder may or may not be committed yet — check
  `git status`. If uncommitted and wanted, commit it (do NOT push unless
  I say so).
- `.gitignore` excludes `test/build/`, `build/`, `selftest/build/`.
- Two zips sit untracked in repo root (`track_purist.zip`,
  `track_purist_fixed.zip`) from an earlier review — harmless.

## 9. History / why things are the way they are

- Code was scaffolded by me, then a second agent found real bugs
  (off-by-one frame size 30 vs 29 edges; decode-on-empty-queue). Fixed.
- A reviewer then flagged doc violations, all fixed: a two-writer race on
  the edge drop counter (Change Requirement 1), over-wide fields
  (narrowed to uint8_t), invented fault bits (removed, deferred to G1
  table), overflow treated as a fault instead of a counter (fixed to
  counter per G3), and `g_line_cfg` having no owner (line task now owns
  cfg + calibrates in READY + refuses to run uncalibrated).
- Design commitments honoured: bit-packed messages (CR1), 20 ms heartbeat
  on barcode wait, zero-timeout queue sends, ISR timestamp-only, decoder
  is sole writer of the barcode byte.

## 10. First thing to do next session

1. Read this file.
2. Ask me the OPEN QUESTIONS (section 6) — especially the Right-IR pin
   and Grove-vs-jumper wiring — BEFORE editing pins or flashing.
3. If wiring is known: edit pins in `selftest/track_purist_selftest.c`
   (lines ~35-38), rebuild, flash, read serial.
4. If a sensor reads garbage or calibration fails -> almost certainly a
   wrong pin / wiring / voltage issue, not the logic.

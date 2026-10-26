# Buddy 3 (Track Purist) — Team Handoff

Owner: Wong Yong Quan (2500377). Scope: IR line following + barcode reading.
Last updated: 2026-10-24. Code: `track_purist/` (branch `buddy3-track-purist`, not yet pushed).

This file is the reference for anyone integrating with the IR subsystem.
It supersedes the IR pin / sensor-count details in the Tribunal 1 doc
(which still says 4 sensors on GP26/GP28/GP6/GP7).

---

## 1. Hardware (verified on the car)

**3 IR sensors** (blue TCRT5000-style modules with A0 + D0 + pot), all on Robo Pico Grove ports, powered at 3V3.

| Sensor | Grove | Wire → module pin | Pico pin | Mode |
|---|---|---|---|---|
| Front left | Grove 6 | yellow → A0 | GP27 / ADC1 | analog, polled every 5 ms |
| Front right | Grove 5 | yellow → A0 | GP26 / ADC0 | analog, polled every 5 ms |
| Barcode (car's LEFT side) | Grove 7 | white → D0 | GP7 | digital, edge interrupt (not fitted yet) |

All sensors: red (3V3) → VCC, black → GND.

Rules / gotchas:
- Grove 5 and Grove 6 **share GP26**. Grove 6's **white** wire must stay unplugged or it shorts against the front-right sensor.
- Grove 5 white (GP6) and Grove 7 yellow (GP28) are unused by Buddy 3 → **GP6 and GP28 are free**. GP28 also feeds the optional VBAT-sense jumper; don't solder it if someone else uses GP28.
- No clash with other buddies' pins: encoders GP0/1 + GP16/17, IMU I2C GP2/3, HC-SR04 GP4/5, motors GP8–11, servos GP12–15, NeoPixel GP18, buttons GP20/21, buzzer GP22.

Physical layout: the two front sensors **straddle** the line (line narrower than the gap between them).

## 2. What Buddy 3 publishes

Two queues, both written only by Buddy 3 tasks, zero-timeout sends (a full queue drops the message, never blocks).

### 2.1 `line_err_q` — one `uint16_t`, every 5 ms (Line Follow task)

| Bits | Field | Meaning |
|---|---|---|
| 0–9 | `error` | Signed 10-bit (two's complement), −512..+511. **+ve = line under LEFT sensor, −ve = under RIGHT sensor, 0 = centred.** Proportional: grows as the line slides under a sensor. |
| 10 | `junction` | 1 = both sensors black for ≥ 30 ms. Held high for as long as both stay black (can't be missed by a slower reader). |
| 11 | `lost` | 1 = line data **not usable** (sensors uncalibrated). Consumer should hold the car. |
| 12 | `last_side` | Which sensor last saw the line: 0 = LEFT, 1 = RIGHT. Kept while both are white. |
| 13–15 | reserved | Always 0. |

How to use it:
- **Both white is normal** (centred). `error = 0`, `lost = 0`. It is *not* "line lost".
- **Bend recovery:** if the car runs off the line on a bend, both sensors go white and `error` returns to 0. Steer toward `last_side` until a sensor sees black again.
- **Junction:** use the rising edge of `junction` to execute the last decoded barcode command.
- Only the newest message matters for steering (G5: Motor Control runs at 20 ms, 4 messages arrive per cycle).
- Time-based "lost" is **disabled** (`lost_timeout_ms = 0`): two straddling sensors can't tell "centred on a long straight" from "off the line". A distance-based lost (using encoder distance) can be added later if the team wants it.

Unpack with the helpers in `include/track_types.h`:
`line_err_get_error()`, `line_err_get_junction()`, `line_err_get_lost()`, `line_err_get_last_side()`
(constants `LINE_SIDE_LEFT = 0`, `LINE_SIDE_RIGHT = 1`).

Examples: centred `0x0000`; line fully under right sensor `0x1201` (err −511, last_side R);
junction after a left-side touch `0x0400`; uncalibrated `0x0800`.

### 2.2 `barcode_q` — one byte, once per barcode read (Barcode Decode task)

| Bits | Field | Meaning |
|---|---|---|
| 0 | `valid` | 1 = a `*<letter>*` barcode decoded OK, 0 = INVALID |
| 1–5 | `letter` | 0 = 'A' … 25 = 'Z' (only meaningful if valid) |
| 6–7 | reserved | 0 |

Helpers: `barcode_is_valid()`, `barcode_get_letter()`, `barcode_get_char()`.

Decoding: Code 39, **every letter A–Z** (prof requirement). Frame `* <letter> *` = 15 bars = 30 edges = 29 widths.
**Either scan direction gives the same letter:** the decoder tries the frame forwards, then backwards; `*` read
backwards is `P`, so only the true direction can pass the start/stop check. Digits/symbols, bad guards,
ambiguous widths or a wrong edge count → INVALID. Pattern table checked against the course samples A, F, W, X, Y, Z.

The decoder sends the **letter, not a movement**. The letter → movement mapping is not hard-coded (see 2.3).

### 2.3 Demo flow + letter → movement mapping over MQTT (`nav_cmd.h`)

Demo: car reads a barcode (either direction) → stores the letter → keeps line following → at the next
**cross-junction** performs the movement mapped to that letter → repeats for the next barcode. Four movements:
LEFT, RIGHT, FORWARD, U-TURN. The instructor announces which 4 letters mean which movement just before the demo;
the PC sends that mapping to the car over MQTT.

**MQTT mapping payload — 3 bytes, binary (Change Requirement 1):**

| Bits (24-bit little-endian) | Letter index (0='A'..25='Z') for |
|---|---|
| 0–4 | LEFT |
| 5–9 | RIGHT |
| 10–14 | FORWARD |
| 15–19 | U-TURN |
| 20–23 | 0 |

`payload[0]` = bits 0–7, `payload[1]` = bits 8–15, `payload[2]` = bits 16–23. The 4 letters must be different.
Example: LEFT=A, RIGHT=B, FORWARD=C, U-TURN=D → `20 88 01`. PC side can use `nav_map_encode()` as the reference.

Who calls what:

| Step | Caller | Function |
|---|---|---|
| Mapping message arrives | Telemetry (Aurelius) | `nav_map_apply(&g_nav_map, payload, len)` → returns `NAV_MAP_OK` / `ERR_LEN` / `ERR_RANGE` / `ERR_DUP`; put that in the ack. A bad payload keeps the old mapping. |
| Each `barcode_q` message | Mission Control (Sean) | `nav_on_barcode(&nav, b)` — valid letter becomes pending (latest wins); INVALID ignored |
| Each `line_err_q` message | Mission Control | `nav_on_line(&nav, le, nav_map_snapshot(&g_nav_map), &move, &letter)` |

`nav_on_line()` returns, on the **rising edge** of `junction` only:
- `NAV_EVT_MOVE` → do `move` (`BARCMD_LEFT/RIGHT/STRAIGHT/U_TURN`), pending cleared.
- `NAV_EVT_UNMAPPED` → a letter was pending but is not one of the 4 mapped letters (or no map received yet). Pending cleared; Mission Control decides (suggest: go forward, report over telemetry).
- `NAV_EVT_NONE` → keep line following (also for a junction with no barcode pending).

Thread safety: the mapping is one aligned 32-bit word, written in a single store by the task that handles MQTT and
read as one snapshot by Mission Control — atomic on the RP2040, no read-modify-write. `nav_state_t` is private to
Mission Control. Topic name and ack format: Aurelius to define.

## 3. Tasks, timing, shared resources

| Item | Value |
|---|---|
| Line Follow period | 5 ms (NFR1); sets its heartbeat bit every loop |
| Barcode Decode | polls the edge queue; when idle sleeps 20 ms and sets its heartbeat bit (never waits without a timeout) |
| Barcode edge queue | 32 timestamps (a barcode needs 30). Overflow: edge dropped and counted, that barcode → INVALID, next one starts clean (G3) |
| Partial barcode timeout | 150 ms of no edges → abandon, report INVALID once |
| Junction hold | 30 ms |

Shared resources:
- **ADC**: used only by the Line Follow task (GP26/GP27). Nobody else should call `adc_select_input()` / `adc_read()`.
- **GPIO interrupt (shared callback)**: the barcode handler is `barcode_edge_isr(void)` — timestamp only. It must be **called from the team's shared GPIO dispatcher** for GP7 (both edges). Do **not** use `gpio_set_irq_enabled_with_callback()` per subsystem: there is only one callback for the whole chip, so a second registration silently replaces the encoder / echo handlers.
- **Heartbeat flag**: Line Follow and Barcode Decode each set their own bit.
- **Fault bits**: Buddy 3 defines none yet. Will use only the bit(s) assigned in Sean's shared fault table (G1), set via event flag.

## 4. What the integration layer must provide

Declared `extern` in `src/line_follow.c` and `src/barcode_decode.c`:

| Symbol | Purpose |
|---|---|
| `g_line_err_q`, `g_barcode_q` | the two queues |
| `g_hb_flag`, `g_line_follow_hb_bit`, `g_barcode_hb_bit` | heartbeat flag + bit assignments (Aurelius) |
| `g_barcode_eq` | the edge queue instance |
| `g_nav_map` (`nav_map_t`) | the shared letter→movement mapping; `nav_map_init()` at boot, written only via `nav_map_apply()` |
| `board_read_left_adc()`, `board_read_center_adc()` | ADC reads (left = GP27/ADC1, "center" = right sensor GP26/ADC0) |
| `board_now_us()` | microsecond timestamp (`time_us_32()`) |
| `board_line_ready()` | 1 while in READY (calibration phase) |
| `board_line_sample_is_line()` | 1 while the operator holds the sensors over the black line, 0 over background |

**Calibration** happens in READY, owned by the Line Follow task: hold both sensors over black, then over background. If calibration fails the task reports `lost = 1` forever instead of steering on garbage. How the operator signals "now on black / now on background" (button, fixed sequence) needs agreeing with Mission Control.

RTOS calls go through `include/rtos_port.h`; `src/rtos_port_utk.c` is currently a **stub** for µT-Kernel.

## 5. Status

Verified:
- Host unit tests pass, 0 failed: line follow 65 checks, barcode 125 checks (all 26 letters forwards + backwards,
  course samples, digits rejected, scale tolerance), nav_cmd 36 checks (MQTT mapping encode/apply/reject, demo flow).
- Self-test firmware builds with and without the barcode sensor enabled (no warnings).
- Bare-metal self-test on the car: both front sensors read correctly over black/white (e.g. right ≈ 150 white / ≈ 2350 black; left has weaker contrast ≈ 1870 / ≈ 2465 — height still being tuned).

Not verified yet:
- Barcode sensor on hardware (not fitted) — decoding verified only in host tests.
- Real MQTT path (needs Aurelius's subscriber to call `nav_map_apply()`).
- Anything under µT-Kernel (stub RTOS port; team risk A7).
- IR noise with Wi-Fi active / WL_GPIO1 HIGH (A5).
- Thresholds under lab lighting and over humps (A2); printed bar widths (G11).

## 6. Open items needing other buddies

| Item | With |
|---|---|
| Accept the `line_err_q` layout above (bit 11 meaning change, new bit 12) | Sean |
| Accept the new `barcode_q` layout (letter, not command) and call `nav_on_barcode()` / `nav_on_line()` | Sean |
| MQTT topic + ack for the 3-byte mapping payload; call `nav_map_apply()`; PC tool to send it | Aurelius |
| Owner of the shared `nav_map_t g_nav_map` instance (suggest: defined in Mission Control, written only by Telemetry) | Sean / Aurelius |
| What to do on `NAV_EVT_UNMAPPED` (suggest: forward + telemetry event) | Sean |
| Shared GPIO dispatcher owner; route GP7 to `barcode_edge_isr()` | Sean / whoever owns the dispatcher |
| Heartbeat bit numbers for the two Buddy 3 tasks | Aurelius |
| Buddy 3 fault bit(s), if any, in the shared fault table | Sean |
| READY-phase calibration signalling | Sean |
| Barcode side (A9: now LEFT) | team / teaching team |
| Fields for the Week 13 data-representation table (G10) | Aurelius |

## 7. How to run

Host tests (laptop, MinGW):
```powershell
cd track_purist\test
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
ctest --test-dir build --output-on-failure
```

Self-test firmware (no RTOS, prints over USB serial at 115200):
```powershell
cd track_purist\selftest
powershell -ExecutionPolicy Bypass -File .\build_uf2.ps1
# flash build\track_purist_selftest.uf2 (hold BOOTSEL, drag onto RPI-RP2)
```
Output line: `L=<adc>(B/W) C=<adc>(B/W) | err junction lost last=L/R raw=0x.... | pending=<letter or ->`
(`C` = front-right sensor.) Barcode reading in the self-test is off until `ENABLE_BARCODE` is set to 1.

Self-test serial commands (type in the serial monitor, then Enter):
- `FZWX` — set the mapping LEFT=F, RIGHT=Z, FORWARD=W, U-TURN=X (goes through the same `nav_map_apply()` as MQTT).
- `#Z` — pretend barcode Z was read (lets you test the junction logic without the barcode sensor).
- Then put both front sensors on black (a junction): it prints `>>> JUNCTION: barcode 'Z' -> RIGHT`.

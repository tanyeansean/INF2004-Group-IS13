# Track Purist (Buddy 3) — Line Following + Barcode Decode

IR sensing subsystem for Group IS13. Raspberry Pi Pico + Robo Pico,
C with the Pico C SDK, micro T-Kernel (uT-Kernel 3.0) RTOS.

## Hardware (3 IR sensors)

| Sensor  | Role                                   | Pin        | Signal                |
|---------|----------------------------------------|------------|-----------------------|
| Left    | Line following (black-line edge)       | GP26 / ADC0| Analog                |
| Center  | Line following (keep robot centered)   | GP28 / ADC2| Analog                |
| Right   | Barcode (right side of track)          | GPx (TBD)  | Digital, edge interrupt |

## Outputs (consumed by Mission Control, Buddy 2)

- `line_err_q` — packed `line_err_t` (13 bits of a `uint16_t`) every 5 ms:
  10-bit signed error + junction flag + lost-line flag.
- `barcode_q` — packed `barcode_t` (valid bit + 2-bit command).

All inter-task / ISR-to-task data is bit-packed per Change Requirement 1.

## Design highlights

- **Line following**: difference of Left and Center analog readings →
  10-bit signed error (0 = centered). Junction = both sensors black held
  for `junction_hold_ms` (configurable, edge-latched so it fires once).
  Lost line = both on background. Junction is **never** derived from the
  Right sensor.
- **Calibration** (`ir_calibrate`): captures Left/Center ADC over line vs
  background in READY, sets thresholds at the midpoint, derives polarity
  (`dark_is_high`). Closes assumption A2.
- **Barcode**: Right-IR edge ISR is **timestamp-only** → 32-entry edge
  ring queue. Overflow drops the edge, counts it, and forces the frame
  INVALID (G3). Decoder assembles a 29-element Code 39 frame
  (15 bars = 30 edges = 29 widths) `* <cmd> *`, classifies each
  9-element character (exactly 3 wide of 9),
  validates both `*` guards, matches the middle char against a 4-entry
  command table, and retries the **reversed** frame for direction
  independence. Anything else (bad guards, unknown char incl. `Z`,
  wrong element count) → INVALID. Decoder wait uses a 20 ms timeout so
  it keeps setting its heartbeat bit.

### Barcode command table (PLACEHOLDER — confirm with team)

| Char | Pattern (9-bit) | Command         | 2-bit |
|------|-----------------|-----------------|-------|
| `A`  | `100001001`     | `BARCMD_LEFT`     | 00  |
| `B`  | `001001001`     | `BARCMD_RIGHT`    | 01  |
| `C`  | `101001000`     | `BARCMD_STRAIGHT` | 10  |
| `D`  | `000011001`     | `BARCMD_U_TURN`   | 11  |
| `*`  | `010010100`     | start/stop guard  | —   |

No STOP command (per spec). `Z` is a valid Code 39 char but not in the
table, so it decodes as INVALID (tested).

## Data representation (Change Requirement 1)

Minimum-bit widths for Buddy 3 application data. Shared/packed fields use
the fewest bits for their range; multi-field packed values have a single
writer so there is no read-modify-write race.

| Field | Range | Bits / type | Where | Notes |
|-------|-------|-------------|-------|-------|
| line error | −512..+511 | 10-bit signed in `uint16_t` | `line_err_q` | packed with the two flags below |
| junction flag | 0/1 | 1 bit | `line_err_q` | held high while both-black |
| lost-line flag | 0/1 | 1 bit | `line_err_q` | 13 bits used of the `uint16_t` |
| barcode valid | 0/1 | 1 bit | `barcode_q` | packed in one `uint8_t` |
| barcode command | 0..3 | 2 bits (`uint8_t`) | `barcode_q` | `barcode_cmd_t` values; carried as `uint8_t`, never a 4-byte enum |
| edge ring head/tail | 0..31 | `uint8_t` | edge queue | single atomic access |
| edge drop counter | per-run count | `uint8_t` | edge queue | **ISR is the sole writer**; consumer reads a delta vs its own last-seen copy (no race, CR1) |
| edge timestamp | raw 32-bit timer | `uint32_t` | edge queue | **intentionally wide**: it is the raw hardware microsecond timer; only differences are used, via wrapping unsigned subtraction, so 32 bits is correct — not an over-wide application field |

Overflow handling (G3): an edge-queue overflow is a **counter**, not a
fault bit. On any drop since the previous frame, that frame is marked
**INVALID**; no fault bit is raised.

## Fault bits (deferred to Sean's table — G1)

This module defines **no concrete fault-bit constants**. Per gap G1 the
team uses one shared `uint16_t` fault mask with a table naming each bit's
owner, latch and clear behaviour, owned by Buddy 2 (Sean). Bit positions
and which faults Buddy 3 owns are his to assign and must not be invented
here. When the table exists, add only Sean's assigned bit(s) as
compile-time constants (not runtime externs) and set them via a kernel
event flag (atomic).

Removed earlier placeholders and why:
- `FAULT_BARCODE_OVF` — contradicted G3 (overflow is a counter).
- `FAULT_LINE_LOST` — line-lost is reported in-band via `line_err_q`.
- `FAULT_IR_UNCAL` — replaced by a structural guard: the line task
  refuses to follow until calibration succeeds.

## RTOS isolation (risk A7)

The decode and line-follow **logic is RTOS-agnostic and host-testable**.
Only `rtos_port_utk.c` touches uT-Kernel, and it is currently a **stub**
with documented TODOs (the SDK + uT-Kernel integration is unverified).
If the team falls back to FreeRTOS or bare-metal, only the adapter
changes — not the logic.

The line task **owns** its `line_follow_cfg_t` (no shared extern): it
calibrates during READY and refuses to run uncalibrated. The integration
layer provides the remaining `extern` globals / board hooks declared in
`line_follow.c` and `barcode_decode.c` (queue & flag ids, ADC/time reads,
and the READY-phase calibration hooks).

## Layout

```
track_purist/
  include/   track_types.h  line_follow.h  barcode_decode.h
             ir_calibrate.h  rtos_port.h
  src/       line_follow.c  barcode_decode.c  ir_calibrate.c
             rtos_port_utk.c  (uT-Kernel adapter STUB)
  test/      test_line_follow.c  test_barcode.c  test_util.h  CMakeLists.txt
  CMakeLists.txt   (target build fragment: add_subdirectory + link track_purist)
```

## Host unit tests

No hardware or RTOS required:

```sh
cd track_purist/test
cmake -S . -B build -G "MinGW Makefiles"   # or your native generator
cmake --build build
ctest --test-dir build --output-on-failure
```

Current status: 142 checks across 2 suites, all passing (109 barcode,
33 line-follow + calibration). The barcode task-loop logic is covered via
a pure, host-testable frame assembler (barcode_asm_t).

## Open items (from the tribunal doc)

- **A2** — thresholds/bar widths must be measured on the real track.
- **A9** — confirmed: barcodes on the right only (single side arm).
- Barcode command mapping and the Right-IR GPIO pin are placeholders;
  confirm with the team and align `line_err_t` bit layout with Buddy 2's
  `line_err_q` unpack code.
- Complete `rtos_port_utk.c` once uT-Kernel is available (A7).

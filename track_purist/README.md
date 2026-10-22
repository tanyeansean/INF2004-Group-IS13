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
  INVALID (G3). Decoder assembles a 30-element Code 39 frame
  `* <cmd> *`, classifies each 9-element character (exactly 3 wide of 9),
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

## RTOS isolation (risk A7)

The decode and line-follow **logic is RTOS-agnostic and host-testable**.
Only `rtos_port_utk.c` touches uT-Kernel, and it is currently a **stub**
with documented TODOs (the SDK + uT-Kernel integration is unverified).
If the team falls back to FreeRTOS or bare-metal, only the adapter
changes — not the logic.

The integration layer must provide the `extern` globals declared at the
bottom of `line_follow.c` / `barcode_decode.c` (queue & flag ids, board
ADC/time hooks).

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

# Handoff to Sean (Buddy 2, Mission Control / Motion) from Buddy 3 (Track Purist)

From: Wong Yong Quan. Date: 2026-10-24. Code: `track_purist/` on branch `buddy3-track-purist`.
Everything below is host-tested (line 65, barcode 125, nav 36 checks, 0 failed).
Full team reference: `TEAM_HANDOFF.md`.

This describes what Buddy 3 has built and what Mission Control receives from it.

---

## 1. `line_err_q` — one `uint16_t` every 5 ms (I send, you read)

| Bits | Field | Meaning |
|---|---|---|
| 0–9 | `error` | signed −512..+511. **+ve = line under LEFT sensor → steer left; −ve = under RIGHT → steer right; 0 = centred.** Proportional |
| 10 | `junction` | both front sensors black ≥ 30 ms; held high while both stay black |
| 11 | `lost` | **line data unusable** (calibration failed) → hold the car. *Changed meaning:* it no longer means "both white" |
| 12 | `last_side` | **new:** 0 = left sensor last saw the line, 1 = right |
| 13–15 | 0 | |

Unpack: `line_err_get_error()`, `line_err_get_junction()`, `line_err_get_lost()`, `line_err_get_last_side()`
(`include/track_types.h`).

How to use it:
- **Both sensors white is normal** — the two front sensors straddle the line, so centred = both white = `error 0`.
- **Bend recovery:** if you overshoot a bend, both go white and `error` returns to 0. If you know you're off the line,
  steer toward `last_side` until a sensor sees black again.
- **Only the newest message matters** (G5): I send every 5 ms, you run every 20 ms. Drain the queue, use the last one —
  but feed **every** message to `nav_on_line()` (§3) so a junction edge is never missed.
- There is **no time-based "lost"**: two straddling sensors can't tell "centred on a long straight" from "off the line".
  If you want a real off-track detector, base it on distance (no sensor near the line for X cm of encoder travel).

Examples: centred `0x0000`; line fully under right sensor `0x1201`; junction `0x0400`; uncalibrated `0x0800`.

## 2. `barcode_q` — one byte per barcode read (I send, you read)

| Bits | Field | Meaning |
|---|---|---|
| 0 | `valid` | 1 = decoded OK |
| 1–5 | `letter` | 0 = 'A' … 25 = 'Z' |
| 6–7 | 0 | |

*Changed:* it carries the **letter**, not a movement (prof: decode all A–Z; the instructor's letter→movement mapping
comes over MQTT at demo time). Either scan direction gives the same letter. Helpers: `barcode_is_valid()`,
`barcode_get_letter()`, `barcode_get_char()`.

## 3. Demo flow — `include/nav_cmd.h` (you call these; pure C, no RTOS)

Demo: read barcode → store letter → keep line following → at the next **cross-junction** do the mapped movement →
repeat for the next barcode.

```c
#include "nav_cmd.h"

extern nav_map_t g_nav_map;   /* shared mapping, written only by Telemetry (Aurelius) */
static nav_state_t nav;       /* yours, private to Mission Control */

/* at start-up (before Telemetry can write the map) */
nav_map_init(&g_nav_map);
nav_reset(&nav);

/* every barcode_q message */
nav_on_barcode(&nav, b);

/* every line_err_q message */
uint8_t move, letter;
switch (nav_on_line(&nav, le, nav_map_snapshot(&g_nav_map), &move, &letter)) {
case NAV_EVT_MOVE:     /* do the move: BARCMD_LEFT / BARCMD_RIGHT / BARCMD_STRAIGHT / BARCMD_U_TURN */ break;
case NAV_EVT_UNMAPPED: /* letter not one of the 4 mapped letters, or no map received yet */ break;
case NAV_EVT_NONE:     /* keep line following */ break;
}
```

Behaviour (all tested):
- Fires once, on the **rising edge** of `junction`; held-high junction does not re-fire.
- Pending letter cleared after it fires (both `MOVE` and `UNMAPPED`).
- Junction with nothing pending → `NONE` (drive on).
- Latest valid barcode before a junction wins; INVALID reads are ignored and don't erase a pending letter.
- Movement codes: `BARCMD_LEFT 0, BARCMD_RIGHT 1, BARCMD_STRAIGHT 2 (forward), BARCMD_U_TURN 3`.
- `g_nav_map` is one aligned 32-bit word written only by Telemetry via `nav_map_apply()`, so the
  `nav_map_snapshot()` read is atomic.

## 4. Other integration points

| Item | Detail |
|---|---|
| Barcode interrupt | `barcode_edge_isr(void)` (timestamp only) must be called from the **shared GPIO dispatcher** for **GP7, both edges**. Don't let any subsystem call `gpio_set_irq_enabled_with_callback()` — one callback per chip, a second one silently replaces the encoder/echo handlers |
| My pins | GP26 (ADC0), GP27 (ADC1), GP7. GP6 and GP28 are free. No clash with your GP0/1, GP16/17, GP8–11 |
| ADC | Only my Line Follow task uses it |
| Calibration (READY) | Line task reads `board_line_ready()` (1 while READY) and `board_line_sample_is_line()` (1 = sensors on black, 0 = background) to collect calibration samples |
| Fault bits | None defined by Buddy 3 |
| Heartbeat | Line Follow and Barcode Decode each set their own bit (numbers from Aurelius) |

## 5. Timing

| Item | Value |
|---|---|
| Line Follow period | 5 ms |
| Junction hold before flag | 30 ms |
| Barcode quiet timeout (car) | 150 ms between bars → partial read abandoned |
| Barcode at 0.2 m/s | ≈ 0.7 s to pass the 141 mm barcode; 3 mm bar ≈ 15 ms |

Barcode decoding compares thin vs thick bar *durations*, so it works best at a roughly constant speed across the
barcode.

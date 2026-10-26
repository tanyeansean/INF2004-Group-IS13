# Handoff to Aurelius (Buddy 1, MQTT / Telemetry) from Buddy 3 (Track Purist)

From: Wong Yong Quan. Date: 2026-10-24. Code: `track_purist/include/nav_cmd.h`, `src/nav_cmd.c`.
Host-tested (36 checks, 0 failed). Full team reference: `TEAM_HANDOFF.md`.

**What I need from you:** subscribe to one MQTT topic, pass its payload to `nav_map_apply()`, ack the result,
and make the PC tool send the mapping in the 3-byte format below.

---

## 1. Why

Demo rules (from the prof): the car reads barcodes A–Z, stores the letter, and at the next cross-junction does one
of 4 movements: **LEFT, RIGHT, FORWARD, U-TURN**. Just before the demo the instructor announces which 4 letters mean
which movement. That mapping must go **PC → car over MQTT**; it must not be hard-coded.

## 2. Payload — 3 bytes, binary (Change Requirement 1: no text)

Each letter is an index 0..25 ('A' = 0 … 'Z' = 25) = 5 bits. Four of them, **always in this order**:

| Bits (24-bit value, little-endian) | Letter for |
|---|---|
| 0–4 | LEFT |
| 5–9 | RIGHT |
| 10–14 | FORWARD |
| 15–19 | U-TURN |
| 20–23 | must be 0 |

```
value   = LEFT | (RIGHT << 5) | (FORWARD << 10) | (UTURN << 15)
payload = [ value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF ]
```

Rules: exactly 3 bytes; the 4 letters must all be different.

Examples (LEFT, RIGHT, FORWARD, U-TURN):

| Mapping | Payload |
|---|---|
| A, B, C, D | `20 88 01` |
| F, Z, W, X | `25 DB 0B` |

⚠ Sending the text `"ABCD"` (bytes `41 42 43 44`) is **rejected** (wrong length).
⚠ Order matters: if the instructor says "A = forward", A goes in the FORWARD slot, not first.

PC-side encoder (Python):
```python
def encode_map(left, right, forward, uturn):
    idx = [ord(c.upper()) - ord('A') for c in (left, right, forward, uturn)]
    if any(not 0 <= i <= 25 for i in idx) or len(set(idx)) != 4:
        raise ValueError("need 4 different letters A-Z")
    v = idx[0] | (idx[1] << 5) | (idx[2] << 10) | (idx[3] << 15)
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])

# encode_map('A','B','C','D') == b'\x20\x88\x01'
```
C reference on the car side: `nav_map_encode()` in `src/nav_cmd.c`.

## 3. Car side — what Telemetry calls

```c
#include "nav_cmd.h"
extern nav_map_t g_nav_map;   /* defined in Mission Control (Sean), init with nav_map_init() at boot */

/* in the Telemetry task, when a message arrives on the mapping topic */
uint8_t st = nav_map_apply(&g_nav_map, payload, payload_len);
/* send st back in the ack */
```

| Return | Value | Meaning |
|---|---|---|
| `NAV_MAP_OK` | 0 | applied |
| `NAV_MAP_ERR_LEN` | 1 | payload not 3 bytes |
| `NAV_MAP_ERR_RANGE` | 2 | a letter > 25, or bits 20–23 not 0 |
| `NAV_MAP_ERR_DUP` | 3 | same letter used for two movements |

- On any error the **previous mapping is kept**.
- The instructor can resend; a new valid payload replaces the old one.
- Call it from the **Telemetry task**, not inside the raw MQTT/lwIP callback (your rule: callbacks only copy bytes).
- Thread safety: `nav_map_apply()` builds the value locally and publishes it with **one aligned 32-bit store**;
  Mission Control reads it with one 32-bit load. Atomic on the RP2040, no read-modify-write, no lock needed —
  as long as **only Telemetry** writes `g_nav_map`.

## 4. To agree / for you to define

| Item | Suggestion |
|---|---|
| Topic name | e.g. `cmd/barcode_map` (your naming scheme) |
| QoS | 1 (it's a command; must not be lost) |
| Ack | your 1-byte ack format with the `nav_map_apply()` return code |
| Telemetry events (optional, from Mission Control) | barcode letter read; junction move executed; `UNMAPPED` letter |
| Heartbeat bits | please assign one each for Line Follow and Barcode Decode |
| G10 data-layout table | rows: map payload (3 B), `barcode_q` (valid 1 b + letter 5 b), `line_err_q` (16 b, see `TEAM_HANDOFF.md`) |

## 5. Testing without MQTT

My self-test firmware takes the same mapping typed on the USB serial monitor (e.g. `ABCD`) and runs it through the
same `nav_map_apply()`, printing `MAP OK ... (payload 20 88 01)`. Use that to check your PC encoder produces the same
bytes.

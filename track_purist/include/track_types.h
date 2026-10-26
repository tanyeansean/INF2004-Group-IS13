/**
 * @file track_types.h
 * @brief Buddy 3 (Track Purist) shared data types.
 *
 * All inter-task / ISR-to-task application data is bit-packed per
 * Change Requirement 1 (Minimum-Bit Data Representation).
 *
 * Hardware (IS13, 3 IR sensors, Robo Pico Grove ports):
 *   - Left  front IR: analog, GP27 / ADC1, Grove 6 (line following)
 *   - Right front IR: analog, GP26 / ADC0, Grove 5 (line following;
 *                     called "center" in the line_follow API)
 *   - Barcode IR    : digital D0, GP7, Grove 7, edge interrupt
 *                     (car's LEFT side)
 *
 * This header is pure C with no SDK / RTOS dependency so it can be
 * compiled on the host for unit testing.
 */
#ifndef TRACK_TYPES_H
#define TRACK_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Movement set (2-bit)                                                */
/* ------------------------------------------------------------------ */
/**
 * The four movements a barcode can trigger at the next cross-junction.
 * Which LETTER means which movement is NOT fixed: it is sent from the PC
 * over MQTT just before the demo (see nav_cmd.h). Exactly 4 values so a
 * movement fits in 2 bits. (Names kept as BARCMD_* for compatibility;
 * BARCMD_STRAIGHT = "forward".)
 */
typedef enum {
    BARCMD_LEFT     = 0u,  /* 0b00 */
    BARCMD_RIGHT    = 1u,  /* 0b01 */
    BARCMD_STRAIGHT = 2u,  /* 0b10  forward */
    BARCMD_U_TURN   = 3u   /* 0b11 */
} barcode_cmd_t;

#define BARCMD_COUNT         4u

/* ------------------------------------------------------------------ */
/* barcode_q message: valid bit + 5-bit letter, packed in one byte     */
/* ------------------------------------------------------------------ */
/**
 * Packed barcode result published to barcode_q.
 *   bit0      : valid  (1 = a Code 39 '*<letter>*' was decoded, 0 = INVALID)
 *   bits1..5  : letter (0 = 'A' .. 25 = 'Z'; only meaningful when valid)
 *   bits6..7  : reserved (keep 0)
 *
 * The decoder reports the LETTER, not a movement: the letter->movement
 * mapping arrives over MQTT at runtime and is applied by nav_cmd.
 *
 * ONE writer rule: only the Barcode Decode task writes this value.
 * The barcode edge ISR never writes it; it only timestamps edges.
 */
typedef struct {
    uint8_t packed; /* [valid:1][letter:5][reserved:2] */
} barcode_t;

#define BARCODE_VALID_MASK     0x01u
#define BARCODE_LETTER_SHIFT   1u
#define BARCODE_LETTER_MASK    0x1Fu
#define BARCODE_LETTER_COUNT   26u   /* A..Z */

/* Pack a decoded letter index (0..25). Out-of-range => INVALID. */
static inline barcode_t barcode_pack_letter(uint8_t letter)
{
    barcode_t b;
    if (letter >= BARCODE_LETTER_COUNT) {
        b.packed = 0u;
    } else {
        b.packed = (uint8_t)(BARCODE_VALID_MASK |
                             ((letter & BARCODE_LETTER_MASK) << BARCODE_LETTER_SHIFT));
    }
    return b;
}

static inline uint8_t barcode_is_valid(barcode_t b)
{
    return (uint8_t)(b.packed & BARCODE_VALID_MASK);
}

/** Letter index 0..25 (A..Z). Only meaningful if barcode_is_valid(). */
static inline uint8_t barcode_get_letter(barcode_t b)
{
    return (uint8_t)((b.packed >> BARCODE_LETTER_SHIFT) & BARCODE_LETTER_MASK);
}

/** ASCII 'A'..'Z' for a valid result, '?' otherwise. */
static inline char barcode_get_char(barcode_t b)
{
    uint8_t l = barcode_get_letter(b);
    if (!barcode_is_valid(b) || l >= BARCODE_LETTER_COUNT) {
        return '?';
    }
    return (char)('A' + l);
}

/* An INVALID result: all bits 0. */
static inline barcode_t barcode_invalid(void)
{
    barcode_t b;
    b.packed = 0u;
    return b;
}

/* ------------------------------------------------------------------ */
/* line_err_q message: 13 bits packed in one uint16_t                  */
/* ------------------------------------------------------------------ */
/**
 * Packed line-following result published to line_err_q every 5 ms.
 *
 * Layout inside a uint16_t (13 bits used, 3 bits reserved):
 *   bits 0..9   : error     (10-bit signed, two's complement, -512..+511;
 *                            +ve = line under LEFT sensor, -ve = under
 *                            RIGHT sensor, 0 = centred / both white)
 *   bit  10     : junction  (1 = both sensors black held >= N ms)
 *   bit  11     : lost_line (1 = line data NOT usable, e.g. uncalibrated
 *                            -> Mission Control should hold the car)
 *   bit  12     : last_side (0 = LEFT sensor last saw the line,
 *                            1 = RIGHT sensor last saw it; use it to
 *                            steer back if both go white after a bend)
 *   bits 13..15 : reserved (keep 0)
 */
typedef struct {
    uint16_t packed;
} line_err_t;

#define LINE_ERR_BITS        10u
#define LINE_ERR_MASK        0x03FFu            /* 10 bits */
#define LINE_ERR_SIGN_BIT    (1u << (LINE_ERR_BITS - 1u)) /* 0x0200 */
#define LINE_JUNCTION_BIT    (1u << 10)
#define LINE_LOST_BIT        (1u << 11)
#define LINE_LAST_SIDE_BIT   (1u << 12)

#define LINE_SIDE_LEFT       0u
#define LINE_SIDE_RIGHT      1u

#define LINE_ERR_MIN         (-512)
#define LINE_ERR_MAX         ( 511)

static inline line_err_t line_err_pack(int16_t error,
                                        uint8_t junction,
                                        uint8_t lost_line)
{
    line_err_t l;
    uint16_t e;

    if (error < LINE_ERR_MIN) {
        error = (int16_t)LINE_ERR_MIN;
    }
    if (error > LINE_ERR_MAX) {
        error = (int16_t)LINE_ERR_MAX;
    }

    e = (uint16_t)((uint16_t)error & LINE_ERR_MASK); /* two's complement low 10 bits */

    l.packed = (uint16_t)(e
             | (junction  ? LINE_JUNCTION_BIT : 0u)
             | (lost_line ? LINE_LOST_BIT     : 0u));
    return l;
}

static inline int16_t line_err_get_error(line_err_t l)
{
    uint16_t e = (uint16_t)(l.packed & LINE_ERR_MASK);
    if (e & LINE_ERR_SIGN_BIT) {
        /* sign-extend from 10 bits to 16 bits */
        e |= (uint16_t)~LINE_ERR_MASK;
    }
    return (int16_t)e;
}

static inline uint8_t line_err_get_junction(line_err_t l)
{
    return (uint8_t)((l.packed & LINE_JUNCTION_BIT) ? 1u : 0u);
}

static inline uint8_t line_err_get_lost(line_err_t l)
{
    return (uint8_t)((l.packed & LINE_LOST_BIT) ? 1u : 0u);
}

/** Return l with the last_side bit set to side (LINE_SIDE_LEFT/RIGHT). */
static inline line_err_t line_err_with_last_side(line_err_t l, uint8_t side)
{
    if (side) {
        l.packed = (uint16_t)(l.packed | LINE_LAST_SIDE_BIT);
    } else {
        l.packed = (uint16_t)(l.packed & (uint16_t)~LINE_LAST_SIDE_BIT);
    }
    return l;
}

static inline uint8_t line_err_get_last_side(line_err_t l)
{
    return (uint8_t)((l.packed & LINE_LAST_SIDE_BIT) ? 1u : 0u);
}

/* ------------------------------------------------------------------ */
/* Fault bits (Buddy 3's contribution to the shared fault bitmask)     */
/* ------------------------------------------------------------------ */
/**
 * DEFERRED to Buddy 2's (Sean's) fault table per gap G1.
 *
 * Version C requires ONE shared uint16_t fault mask with a table naming,
 * for each bit: its owner, whether it latches, and how it clears. The
 * mask already holds ~10 faults, so every extra bit costs the team. The
 * bit positions, and even which faults Buddy 3 owns, are Sean's to
 * assign — they must NOT be invented here.
 *
 * Accordingly this module defines no concrete fault-bit constants yet.
 * When Sean publishes the table, add only the bit(s) he assigns to
 * Buddy 3 as compile-time constants (not runtime externs), and set them
 * via a kernel event flag (atomic; never read-modify-write the mask).
 *
 * Note on overflow: edge-queue overflow is NOT a fault bit. Per gap G3
 * it is handled as a per-run COUNTER (drop the edge, count it, mark the
 * barcode INVALID) — see barcode_eq_overflow_delta(). A previous
 * FAULT_BARCODE_OVF bit was removed because it contradicted G3.
 *
 * Line-lost is reported in-band through line_err_q's lost-line flag, so
 * it does not need a separate fault bit unless Sean's table adds one.
 * "Running uncalibrated" is prevented structurally by the line task
 * (it refuses to run until ir_cal_apply succeeds), not by a fault bit.
 */

#ifdef __cplusplus
}
#endif

#endif /* TRACK_TYPES_H */

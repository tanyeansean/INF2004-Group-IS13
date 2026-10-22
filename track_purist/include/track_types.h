/**
 * @file track_types.h
 * @brief Buddy 3 (Track Purist) shared data types.
 *
 * All inter-task / ISR-to-task application data is bit-packed per
 * Change Requirement 1 (Minimum-Bit Data Representation).
 *
 * Hardware (IS13, 3 IR sensors):
 *   - Left  IR : analog, GP26 / ADC0  (line following)
 *   - Center IR: analog, GP28 / ADC2  (line following)
 *   - Right IR : digital, edge interrupt (barcode, right side of track)
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
/* Barcode command set (2-bit)                                         */
/* ------------------------------------------------------------------ */
/**
 * Decoded barcode command. PLACEHOLDER mapping — confirm the physical
 * barcode-to-command assignment with the team before the demo.
 * Exactly 4 values so it fits in 2 bits. There is deliberately no STOP.
 */
typedef enum {
    BARCMD_LEFT     = 0u,  /* 0b00 */
    BARCMD_RIGHT    = 1u,  /* 0b01 */
    BARCMD_STRAIGHT = 2u,  /* 0b10 */
    BARCMD_U_TURN   = 3u   /* 0b11 */
} barcode_cmd_t;

/* ------------------------------------------------------------------ */
/* barcode_q message: valid bit + 2-bit command, packed in one byte    */
/* ------------------------------------------------------------------ */
/**
 * Packed barcode result published to barcode_q.
 *   bit0      : valid  (1 = a barcode was decoded, 0 = INVALID)
 *   bits1..2  : cmd    (barcode_cmd_t, only meaningful when valid == 1)
 * Remaining bits reserved (keep 0).
 *
 * ONE writer rule: only the Barcode Decode task writes this value.
 * The Right-IR edge ISR never writes it; it only timestamps edges.
 */
typedef struct {
    uint8_t packed; /* [valid:1][cmd:2][reserved:5] */
} barcode_t;

#define BARCODE_VALID_MASK   0x01u
#define BARCODE_CMD_SHIFT     1u
#define BARCODE_CMD_MASK     0x03u

/*
 * cmd is carried as uint8_t (0..3), not the enum type, so no 4-byte
 * enum is used as a stored value, parameter, or return (Change
 * Requirement 1, minimum bits). The BARCMD_* names remain the documented
 * value set.
 */
static inline barcode_t barcode_pack(uint8_t valid, uint8_t cmd)
{
    barcode_t b;
    b.packed = (uint8_t)((valid & BARCODE_VALID_MASK) |
                         ((cmd & BARCODE_CMD_MASK) << BARCODE_CMD_SHIFT));
    return b;
}

static inline uint8_t barcode_is_valid(barcode_t b)
{
    return (uint8_t)(b.packed & BARCODE_VALID_MASK);
}

static inline uint8_t barcode_get_cmd(barcode_t b)
{
    return (uint8_t)((b.packed >> BARCODE_CMD_SHIFT) & BARCODE_CMD_MASK);
}

/* An INVALID result: valid = 0, cmd = 0. */
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
 *   bits 0..9   : error   (10-bit signed, two's complement, -512..+511)
 *   bit  10     : junction  (1 = both sensors black held >= N ms)
 *   bit  11     : lost_line (1 = both sensors on background/white)
 *   bit  12     : reserved-for-future flag (keep 0)
 *   bits 13..15 : reserved (keep 0)
 *
 * error is derived from (left_adc - center_adc); 0 means centered on
 * the followed edge. Sign convention: positive = drifted one way,
 * negative = the other (fixed in line_follow.c, documented there).
 */
typedef struct {
    uint16_t packed;
} line_err_t;

#define LINE_ERR_BITS        10u
#define LINE_ERR_MASK        0x03FFu            /* 10 bits */
#define LINE_ERR_SIGN_BIT    (1u << (LINE_ERR_BITS - 1u)) /* 0x0200 */
#define LINE_JUNCTION_BIT    (1u << 10)
#define LINE_LOST_BIT        (1u << 11)

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

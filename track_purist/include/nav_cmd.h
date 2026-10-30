/**
 * @file nav_cmd.h
 * @brief Navigation command interface (Buddy 3): barcode letter + MQTT
 *        letter->movement mapping + cross-junction trigger.
 *
 * Demo flow this implements:
 *   1. Barcode Decode reports a letter A..Z (either scan direction).
 *   2. The letter is stored as PENDING.
 *   3. The car keeps line following until a cross-junction (rising edge
 *      of the line_err_q junction flag).
 *   4. At the junction the pending letter is looked up in the mapping
 *      received over MQTT and the mapped movement is returned (once).
 *   5. Pending is cleared; the next barcode repeats the process.
 *
 * The letter->movement mapping is NOT hard-coded: the instructor's four
 * letters arrive from the PC over MQTT just before the demo and are
 * applied with nav_map_apply().
 *
 * Everything here is pure C (no RTOS / SDK) and host-tested.
 */
#ifndef NAV_CMD_H
#define NAV_CMD_H

#include <stdint.h>
#include "track_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Letter -> movement mapping (received over MQTT)                     */
/* ------------------------------------------------------------------ */
/**
 * MQTT payload: 3 bytes (24 bits), little-endian, Change Requirement 1
 * (compact binary, no text):
 *   bits  0..4  : letter index (0='A'..25='Z') for BARCMD_LEFT
 *   bits  5..9  : letter for BARCMD_RIGHT
 *   bits 10..14 : letter for BARCMD_STRAIGHT (forward)
 *   bits 15..19 : letter for BARCMD_U_TURN
 *   bits 20..23 : 0
 * i.e. payload[0] = bits 0..7, payload[1] = bits 8..15, payload[2] = 16..23.
 * The four letters must be distinct.
 */
#define NAV_MAP_PAYLOAD_LEN   3u
#define NAV_MAP_LETTER_BITS   5u
#define NAV_MAP_LETTER_MASK   0x1Fu
#define NAV_MAP_VALID_BIT     (1uL << 20)  /* in the stored word only */

/** Result of applying a mapping payload (reported back in the MQTT ack). */
#define NAV_MAP_OK            0u
#define NAV_MAP_ERR_LEN       1u  /* payload is not 3 bytes */
#define NAV_MAP_ERR_RANGE     2u  /* a letter index > 25, or bits 20..23 set */
#define NAV_MAP_ERR_DUP       3u  /* the same letter given for two movements */

/**
 * Stored mapping. Written by ONE task (the one that handles the MQTT
 * message) with a single aligned 32-bit store, read by Mission Control
 * with a single 32-bit load. On the RP2040 an aligned word access is
 * atomic, so the reader never sees half an update and there is no
 * read-modify-write on shared data (Change Requirement 1).
 *   bits 0..19 : the 4 letters as in the payload
 *   bit  20    : 1 = a mapping has been received
 */
typedef struct {
    volatile uint32_t packed;
} nav_map_t;

/** Start with no mapping (every movement unassigned). */
void nav_map_init(nav_map_t *m);

/**
 * Validate a payload and, on success, store it atomically. On any error
 * the previous mapping is kept unchanged. Returns NAV_MAP_OK or an error.
 */
uint8_t nav_map_apply(nav_map_t *m, const uint8_t *payload, uint32_t len);

/** Single atomic snapshot of the mapping (use one snapshot per decision). */
uint32_t nav_map_snapshot(const nav_map_t *m);

/** 1 if a mapping has been received. */
uint8_t nav_map_is_set(uint32_t snapshot);

/**
 * Find the movement for a letter index (0..25). Returns 1 and sets *move
 * (BARCMD_*) if the letter is one of the four mapped letters, else 0.
 */
uint8_t nav_map_lookup(uint32_t snapshot, uint8_t letter, uint8_t *move);

/**
 * Build a payload from four letter indices given in BARCMD_* order
 * (left, right, forward, u-turn). For the PC side / tests. Returns
 * NAV_MAP_OK or NAV_MAP_ERR_RANGE / NAV_MAP_ERR_DUP.
 */
uint8_t nav_map_encode(const uint8_t letters[BARCMD_COUNT],
                       uint8_t payload[NAV_MAP_PAYLOAD_LEN]);

/* ------------------------------------------------------------------ */
/* Pending barcode + junction trigger                                  */
/* ------------------------------------------------------------------ */
/** What nav_on_line() tells the caller to do this step. */
#define NAV_EVT_NONE       0u  /* keep line following */
#define NAV_EVT_MOVE       1u  /* junction + pending letter mapped: do *move */
#define NAV_EVT_UNMAPPED   2u  /* junction + pending letter has no mapping */

/** State owned by the consumer task (Mission Control). */
typedef struct {
    barcode_t pending;       /* last VALID barcode not yet executed (invalid = none) */
    uint8_t   prev_junction; /* junction flag seen on the previous step */
} nav_state_t;

void nav_reset(nav_state_t *s);

/**
 * Feed a barcode_q result. A VALID letter replaces any pending letter
 * (the latest barcode before a junction wins). INVALID is ignored.
 */
void nav_on_barcode(nav_state_t *s, barcode_t b);

/**
 * Feed every line_err_q message. On the RISING edge of the junction
 * flag, if a letter is pending:
 *   - mapped   -> returns NAV_EVT_MOVE, *move = BARCMD_*, *letter = index
 *   - unmapped -> returns NAV_EVT_UNMAPPED, *letter = index
 * and clears pending. Otherwise returns NAV_EVT_NONE.
 * A junction with nothing pending returns NAV_EVT_NONE (drive on).
 */
uint8_t nav_on_line(nav_state_t *s, line_err_t le, uint32_t map_snapshot,
                    uint8_t *move, uint8_t *letter);

/** 1 if a letter is waiting for the next junction (and its index). */
uint8_t nav_has_pending(const nav_state_t *s, uint8_t *letter);

#ifdef __cplusplus
}
#endif

#endif /* NAV_CMD_H */

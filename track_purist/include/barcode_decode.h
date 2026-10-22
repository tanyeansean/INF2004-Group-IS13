/**
 * @file barcode_decode.h
 * @brief Barcode Decode module (Buddy 3) — Right digital IR, edge timestamps.
 *
 * Pipeline:
 *   Right-IR edge ISR  -> timestamp only -> 32-entry edge queue
 *   Decode task        -> read edges -> build 30-edge
 *                         Code 39 frame -> decode -> barcode_q
 *
 * Code 39 frame model used here:
 *   - A single Code 39 character = 9 elements (5 bars + 4 spaces),
 *     of which exactly 3 are WIDE and 6 are NARROW. Characters are
 *     separated by a narrow inter-character gap.
 *   - This decoder expects a 3-character frame:  *  <CMD>  *
 *     '*' is the Code 39 start/stop guard.
 *   - 3 chars x 9 elements + 2 inter-character gaps = 29 elements.
 *     That is 15 bars = 30 edges = 29 measured widths. There is NO
 *     30th element: after the last bar the sensor sees plain paper, so
 *     no further edge arrives.
 *   - valid bit is set only when BOTH guards decode as '*'.
 *   - The middle character is matched against a 4-entry command table.
 *   - If the forward frame fails, the REVERSED frame is tried (the car
 *     may read the barcode from either travel direction).
 *   - Anything else (bad guards, unknown middle char including 'Z', or a
 *     frame that is not 29 elements) => INVALID.
 *
 * The decode logic is pure and host-testable: see barcode_decode_frame().
 */
#ifndef BARCODE_DECODE_H
#define BARCODE_DECODE_H

#include <stdint.h>
#include "track_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Edge queue sizing (G3) ---- */
#define BARCODE_EDGE_QUEUE_LEN   32u  /* a full frame is 30 edges */
#define BARCODE_FRAME_EDGES      30u  /* edges in one full barcode (15 bars x 2) */
#define BARCODE_FRAME_ELEMENTS   29u  /* element widths per frame (edges - 1)    */
#define BARCODE_CHAR_ELEMENTS     9u  /* Code 39: 9 elements per char */
#define BARCODE_WIDE_PER_CHAR     3u  /* Code 39: 3 wide elements per char */

/* ---- Decode-task timing ---- */
#define BARCODE_FRAME_TIMEOUT_MS 20u  /* poll period; keeps heartbeat alive */

/*
 * Quiet timeout: if no new edge arrives for this long while a frame is
 * partially assembled, the partial frame is abandoned and reported
 * INVALID once. It must exceed the longest gap between two edges of a
 * valid barcode at the slowest supported run speed.
 *
 * Slowest supported speed ~0.1 m/s. A wide bar is ~9 mm, so the longest
 * single element lasts ~9 mm / 100 mm/s = ~90 ms. 150 ms leaves margin
 * above that worst-case element while still abandoning a truly stalled
 * read quickly. Re-tune against measured traversal time on the real
 * track (open item A2).
 */
#define BARCODE_QUIET_TIMEOUT_MS 150u

/**
 * One captured edge: a timestamp only. The ISR pushes these; it performs
 * no interpretation (keeps the shared GPIO callback fast, and avoids the
 * two-writer race on the barcode value).
 */
typedef struct {
    uint32_t t_us;  /* edge timestamp in microseconds */
} barcode_edge_t;

/**
 * Convert 29 element WIDTHS (durations, any consistent unit) into a
 * decoded barcode result. This is the pure host-testable core.
 *
 *   widths[0..28] : element durations in temporal order of capture.
 *
 * Returns a packed barcode_t: valid + command, or barcode_invalid().
 * Tries the forward frame first, then the reversed frame.
 */
barcode_t barcode_decode_frame(const uint32_t widths[BARCODE_FRAME_ELEMENTS]);

/**
 * Convert an ordered list of edge timestamps into element widths.
 * With N edges you get N-1 intervals. A valid frame needs exactly
 * BARCODE_FRAME_ELEMENTS (29) widths, i.e. BARCODE_FRAME_EDGES (30)
 * edges. Returns the number of widths written (capped at
 * BARCODE_FRAME_ELEMENTS). Helper for the task layer/tests.
 */
uint32_t barcode_edges_to_widths(const barcode_edge_t *edges, uint32_t n_edges,
                                 uint32_t widths_out[BARCODE_FRAME_ELEMENTS]);

/**
 * Classify the 9 elements of a single character into a 9-bit wide/narrow
 * pattern (MSB = first element). "Wide" = the 3 widest of the 9 elements.
 * Returns 1 on success (exactly 3 wide), 0 if the pattern is ambiguous
 * (cannot pick exactly 3 wide cleanly) -> caller treats as INVALID.
 */
uint8_t barcode_classify_char(const uint32_t elems[BARCODE_CHAR_ELEMENTS],
                              uint16_t *pattern9);

/**
 * Map a 9-bit Code 39 element pattern to an ASCII character.
 * Recognises only '*' (guard) and the 4 command characters in the table.
 * Returns the ASCII char, or 0 if not recognised.
 */
char barcode_pattern_to_char(uint16_t pattern9);

/**
 * Map a decoded command CHARACTER to a barcode_cmd_t.
 * Returns 1 and sets *cmd on success, 0 if the char is not a command.
 */
uint8_t barcode_char_to_cmd(char c, barcode_cmd_t *cmd);

/* ---- Edge queue (ISR producer / task consumer), ring buffer ---- */
typedef struct {
    barcode_edge_t buf[BARCODE_EDGE_QUEUE_LEN];
    volatile uint32_t head;      /* ISR writes */
    volatile uint32_t tail;      /* task reads */
    volatile uint32_t dropped;   /* overflow count (G3) */
} barcode_edge_queue_t;

void     barcode_eq_init(barcode_edge_queue_t *q);
/** ISR-side push. Returns 1 if stored, 0 if full (increments dropped). */
uint8_t  barcode_eq_push(barcode_edge_queue_t *q, uint32_t t_us);
/** Task-side pop. Returns 1 if an edge was read, 0 if empty. */
uint8_t  barcode_eq_pop(barcode_edge_queue_t *q, barcode_edge_t *out);
/** Current number of queued edges. */
uint32_t barcode_eq_count(const barcode_edge_queue_t *q);

#ifndef BARCODE_HOST_TEST
#include "rtos_port.h"
/** Right-IR edge ISR: timestamp-only, push to edge queue. */
void barcode_edge_isr(void);
/** Barcode Decode task body: assemble frames, decode, publish barcode_q. */
void barcode_decode_task(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BARCODE_DECODE_H */

/**
 * @file barcode_decode.h
 * @brief Barcode Decode module (Buddy 3) — barcode digital IR, edge timestamps.
 *
 * Pipeline:
 *   Barcode-IR edge ISR -> timestamp only -> 32-entry edge queue
 *   Decode task         -> read edges -> build 30-edge
 *                          Code 39 frame -> decode letter -> barcode_q
 *
 * Code 39 frame model used here:
 *   - A single Code 39 character = 9 elements (5 bars + 4 spaces),
 *     of which exactly 3 are WIDE and 6 are NARROW. Characters are
 *     separated by a narrow inter-character gap.
 *   - This decoder expects a 3-character frame:  *  <LETTER>  *
 *     '*' is the Code 39 start/stop guard; LETTER is any of A..Z.
 *   - 3 chars x 9 elements + 2 inter-character gaps = 29 elements.
 *     That is 15 bars = 30 edges = 29 measured widths. There is NO
 *     30th element: after the last bar the sensor sees plain paper, so
 *     no further edge arrives.
 *   - valid bit is set only when BOTH guards decode as '*'.
 *   - If the forward frame fails, the REVERSED frame is tried (the car
 *     may read the barcode from either travel direction). '*' reversed
 *     reads as 'P', so only the true direction can pass the guards and
 *     both directions report the SAME letter.
 *   - Anything else (bad guards, digits/symbols, ambiguous widths, or a
 *     frame that is not 29 elements) => INVALID.
 *   - The decoder reports the LETTER only. Letter -> movement is applied
 *     later by nav_cmd using the mapping received over MQTT.
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
 * Size- and speed-independent decoding. No rule below uses an absolute
 * bar width, so the barcode may be printed at any size up to A4 and
 * scanned by the car or by hand at any steady speed.
 *
 * Chatter (pulse-pair cancellation): two edges less than
 * BARCODE_CHATTER_US apart are a glitch, not a bar, and BOTH are dropped
 * (a lone dropped edge would swap every later bar and space). At the
 * fastest supported scan, 0.3 m/s, the narrowest real element is a
 * 1.5 mm bar (half-size print) = 5 ms, and the spec 3 mm bar = 10 ms,
 * so 2 ms never touches a real element. Every edge waits up to 2 ms
 * for this check before it counts.
 */
#define BARCODE_CHATTER_US       2000u

/*
 * Abort rule for a partial frame: give up when the current gap exceeds
 * BARCODE_ABORT_FACTOR x the longest element in the frame, clamped to
 * [FLOOR, CAP]. With no element yet (0 or 1 edge) the CAP applies.
 * 6x: a wide element is 3x a narrow one, so after only narrow elements
 * the next (wide) one may still slow by 2x before the read is dropped.
 * 150 ms floor: a fast scan of noise is dropped quickly but not before
 *   a slow-start element can arrive.
 * 2 s cap: the longest pause allowed inside a barcode (slow hand scan).
 * The abort is only a safety net: a frame is decoded on its 30th edge.
 */
#define BARCODE_ABORT_FACTOR     6u
#define BARCODE_ABORT_FLOOR_US   150000u
#define BARCODE_ABORT_CAP_US     2000000u

/*
 * Wide/narrow split: in each character the 3 widest elements are wide.
 * The split is accepted only if the narrowest wide element is at least
 * 1.5x the widest narrow one (spec bars are 3:1, so this still holds with
 * a +/-20% speed change inside one character: 2.4 / 1.2 = 2.0). The two
 * inter-character gaps must pass the same test as narrow elements: the
 * narrowest wide element of BOTH neighbouring characters must be at
 * least 1.5x the gap. Anything else is INVALID, never a guess.
 */
#define BARCODE_SPLIT_RATIO_NUM  3u   /* 1.5 = 3 / 2 */
#define BARCODE_SPLIT_RATIO_DEN  2u

/** Why the assembler produced its last result (debug print). */
typedef enum {
    BARCODE_OUT_NONE = 0,     /* no result yet                           */
    BARCODE_OUT_VALID,        /* 30-edge window decoded                   */
    BARCODE_OUT_PARTIAL,      /* went quiet with fewer than 30 edges      */
    BARCODE_OUT_UNDECODABLE   /* went quiet; 30+ edges, no window decoded */
} barcode_outcome_t;

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

/* ------------------------------------------------------------------ */
/* Pure frame assembler (host-testable, no RTOS, no board hooks)       */
/* ------------------------------------------------------------------ */
/**
 * State machine that turns a stream of edge timestamps into decoded
 * results. It is driven by two calls, both taking the relevant
 * timestamp as an argument so there is no hidden clock dependency:
 *
 *   barcode_asm_edge_arrived(a, t_us)  - an edge was captured at t_us
 *   barcode_asm_time_passed(a, now_us) - no edge; current time is now_us
 *
 * Each call may produce at most one result (valid or invalid) via *out.
 * BOTH calls can produce a VALID result: an edge only counts once the
 * 2 ms chatter check has passed (by the next edge or by time_passed),
 * so the caller must call time_passed regularly (every <= 20 ms).
 *
 * Rules (numbers in the defines above):
 *  1. Chatter: an edge < BARCODE_CHATTER_US after the previous pending
 *     edge cancels both of them (pulse-pair cancellation).
 *  2. Rolling window: the last 30 accepted edges are kept. Every accepted
 *     edge from the 30th on tries to decode the window; stray edges
 *     before a barcode slide out instead of shifting it for good.
 *     A failed window is NOT reported (the next edge may fix it).
 *  3. Decode: the first window that decodes VALID is reported and the
 *     window is cleared.
 *  4. Abort: on a gap longer than the adaptive timeout the window is
 *     reported INVALID once and cleared.
 *
 * The caller still owns overflow handling: on a detected queue overflow
 * it should force INVALID. The assembler itself never touches the queue.
 */
typedef struct {
    uint32_t edges[BARCODE_FRAME_EDGES]; /* accepted edges, oldest first   */
    uint32_t pending_us;      /* newest edge, waiting for the chatter check */
    uint32_t last_edge_us;    /* time of the newest raw edge (abort timer)  */
    uint16_t burst_edges;     /* accepted edges since the last result       */
    uint16_t chatter_pairs;   /* pulse pairs cancelled since the last result */
    uint16_t last_burst_edges;   /* the same two, frozen at the last result */
    uint16_t last_chatter_pairs;
    uint8_t  count;           /* accepted edges in the window, 0..30        */
    uint8_t  have_pending;    /* 1 = pending_us holds an edge               */
    uint8_t  last_count;      /* edges in the window at the last result     */
    uint8_t  last_outcome;    /* barcode_outcome_t of the last result       */
} barcode_asm_t;

/** Initialise (or reset) the assembler: empty window, nothing pending. */
void barcode_asm_init(barcode_asm_t *a);

/**
 * Feed one captured edge. May accept the previous pending edge, which can
 * complete a decodable 30-edge window: then writes *out and returns 1.
 * Otherwise returns 0.
 */
uint8_t barcode_asm_edge_arrived(barcode_asm_t *a, uint32_t t_us, barcode_t *out);

/**
 * Advance time with no edge. Accepts a pending edge once it is
 * BARCODE_CHATTER_US old (this can produce a VALID result), or abandons a
 * partial frame that has gone quiet longer than the adaptive timeout
 * (INVALID, reported once). Writes *out and returns 1 if either happened.
 */
uint8_t barcode_asm_time_passed(barcode_asm_t *a, uint32_t now_us, barcode_t *out);

/** Current adaptive abort timeout in microseconds (rule 4). */
uint32_t barcode_asm_abort_timeout_us(const barcode_asm_t *a);

/** Snapshot of the window behind the last result, for the debug print. */
typedef struct {
    uint32_t widths_us[BARCODE_FRAME_ELEMENTS];
    uint32_t longest_us;      /* longest element in the window            */
    uint16_t burst_edges;     /* edges accepted in this burst (may be > 30) */
    uint16_t chatter_pairs;   /* glitch pulses cancelled in this burst    */
    uint8_t  n_edges;         /* edges in the window (30 when VALID)      */
    uint8_t  n_widths;        /* n_edges - 1, at most 29                  */
    uint8_t  outcome;         /* barcode_outcome_t                        */
} barcode_frame_info_t;

/**
 * Fill *p_info with the window that produced the last result. Call it
 * straight after edge_arrived/time_passed returned 1: the next accepted
 * edge overwrites the window. Uses only the caller's buffer.
 */
void barcode_asm_last_frame(const barcode_asm_t *a, barcode_frame_info_t *p_info);

/**
 * Classify the 9 elements of a single character into a 9-bit wide/narrow
 * pattern (MSB = first element). The 3 widest elements are wide; the
 * split must be clean (narrowest wide >= 1.5 x widest narrow).
 * Returns 1 on success, 0 if ambiguous -> caller treats as INVALID.
 */
uint8_t barcode_classify_char(const uint32_t elems[BARCODE_CHAR_ELEMENTS],
                              uint16_t *pattern9);

/**
 * Map a 9-bit Code 39 element pattern to an ASCII character.
 * Recognises '*' (guard) and the letters 'A'..'Z'.
 * Returns the ASCII char, or 0 if not recognised (digits/symbols too).
 */
char barcode_pattern_to_char(uint16_t pattern9);

/** 9-bit Code 39 pattern for letter index 0..25 ('A'..'Z'); 0 if out of range. */
uint16_t barcode_letter_pattern(uint8_t letter);

/* ---- Edge queue (ISR producer / task consumer), ring buffer ---- */
/*
 * Change Requirement 1 (minimum bits + safe shared access):
 *   - head/tail index a 32-entry ring, so they need only need to count
 *     0..31: uint8_t is sufficient and is a single atomic access on the
 *     RP2040 (one load/store, no read-modify-write tearing across the
 *     ISR/task boundary).
 *   - dropped is a per-run overflow COUNTER (G3: "drop the edge, count
 *     it", NOT a fault bit). It is written ONLY by the ISR (push). The
 *     consumer never writes it; instead it keeps its own last-seen copy
 *     and reads the difference (see barcode_eq_overflow_delta). This
 *     removes the two-writer lost-update race that CR1 forbids.
 */
typedef struct {
    barcode_edge_t buf[BARCODE_EDGE_QUEUE_LEN];
    volatile uint8_t head;       /* ISR writes, task reads (0..31) */
    volatile uint8_t tail;       /* task writes, ISR reads (0..31) */
    volatile uint8_t dropped;    /* ISR is the SOLE writer; G3 counter  */
} barcode_edge_queue_t;

void     barcode_eq_init(barcode_edge_queue_t *q);
/** ISR-side push. Returns 1 if stored, 0 if full (increments dropped). */
uint8_t  barcode_eq_push(barcode_edge_queue_t *q, uint32_t t_us);
/** Task-side pop. Returns 1 if an edge was read, 0 if empty. */
uint8_t  barcode_eq_pop(barcode_edge_queue_t *q, barcode_edge_t *out);
/** Current number of queued edges. */
uint32_t barcode_eq_count(const barcode_edge_queue_t *q);
/**
 * Consumer-side overflow check WITHOUT writing the queue. The task holds
 * its own last-seen drop count in *last_seen; this returns how many new
 * drops occurred since then and updates *last_seen. The ISR remains the
 * only writer of q->dropped (no shared read-modify-write). uint8_t wrap
 * is handled by modular subtraction.
 */
uint8_t  barcode_eq_overflow_delta(const barcode_edge_queue_t *q,
                                   uint8_t *last_seen);

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

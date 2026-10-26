/**
 * @file barcode_decode.c
 * @brief Barcode Decode implementation (Buddy 3).
 *
 * Pure decode core (barcode_decode_frame and helpers) is host-testable.
 * Edge queue is a lock-free-ish SPSC ring (single ISR producer, single
 * task consumer). Task body + ISR are target-only.
 *
 * Code 39 element pattern encoding used here:
 *   9 elements per character, MSB (bit 8) = first element.
 *   1 = WIDE element, 0 = NARROW element. Exactly 3 wide per character.
 *   Element order is bar,space,bar,space,bar,space,bar,space,bar.
 */
#include "barcode_decode.h"

/* ------------------------------------------------------------------ */
/* Code 39 pattern table (9-bit, MSB = first element)                  */
/* ------------------------------------------------------------------ */
/* '*' start/stop guard. Read backwards it is 'P' (0x052), so a frame
   whose guards are both '*' identifies the scan direction uniquely. */
#define PAT_STAR  0x094u /* 0 1001 0100 */

/*
 * Code 39 letters A..Z (ISO/IEC 16388). Index 0 = 'A'.
 * Each pattern has exactly 3 wide elements (2 wide bars + 1 wide space).
 * Verified against the course samples: A, F, W, X, Y, Z.
 */
static const uint16_t k_letter_pat[BARCODE_LETTER_COUNT] = {
    0x109u, /* A 100001001 */
    0x049u, /* B 001001001 */
    0x148u, /* C 101001000 */
    0x019u, /* D 000011001 */
    0x118u, /* E 100011000 */
    0x058u, /* F 001011000 */
    0x00Du, /* G 000001101 */
    0x10Cu, /* H 100001100 */
    0x04Cu, /* I 001001100 */
    0x01Cu, /* J 000011100 */
    0x103u, /* K 100000011 */
    0x043u, /* L 001000011 */
    0x142u, /* M 101000010 */
    0x013u, /* N 000010011 */
    0x112u, /* O 100010010 */
    0x052u, /* P 001010010 */
    0x007u, /* Q 000000111 */
    0x106u, /* R 100000110 */
    0x046u, /* S 001000110 */
    0x016u, /* T 000010110 */
    0x181u, /* U 110000001 */
    0x0C1u, /* V 011000001 */
    0x1C0u, /* W 111000000 */
    0x091u, /* X 010010001 */
    0x190u, /* Y 110010000 */
    0x0D0u  /* Z 011010000 */
};

char barcode_pattern_to_char(uint16_t pattern9)
{
    uint8_t i;
    pattern9 &= 0x1FFu;
    if (pattern9 == PAT_STAR) {
        return '*';
    }
    for (i = 0u; i < BARCODE_LETTER_COUNT; i++) {
        if (k_letter_pat[i] == pattern9) {
            return (char)('A' + i);
        }
    }
    return 0; /* digits / symbols / garbage: not accepted */
}

uint16_t barcode_letter_pattern(uint8_t letter)
{
    if (letter >= BARCODE_LETTER_COUNT) {
        return 0u;
    }
    return k_letter_pat[letter];
}

/* ------------------------------------------------------------------ */
/* Width classification                                                */
/* ------------------------------------------------------------------ */
/*
 * Pick exactly 3 wide elements out of 9 by thresholding at the midpoint
 * between the min and max element width. Code 39 wide:narrow is ~3:1 so
 * there is a clear gap. If the split does not yield exactly 3 wide, the
 * character is ambiguous -> caller treats the frame as INVALID.
 */
uint8_t barcode_classify_char(const uint32_t elems[BARCODE_CHAR_ELEMENTS],
                              uint16_t *pattern9)
{
    uint32_t mn = elems[0], mx = elems[0];
    uint32_t threshold;
    uint16_t pat = 0u;
    uint8_t wide_count = 0u;
    uint8_t i;

    for (i = 1u; i < BARCODE_CHAR_ELEMENTS; i++) {
        if (elems[i] < mn) {
            mn = elems[i];
        }
        if (elems[i] > mx) {
            mx = elems[i];
        }
    }
    if (mx == mn) {
        return 0u; /* all equal: cannot distinguish wide/narrow */
    }
    /* midpoint threshold; wide if strictly above it */
    threshold = mn + (mx - mn) / 2u;

    for (i = 0u; i < BARCODE_CHAR_ELEMENTS; i++) {
        pat = (uint16_t)(pat << 1);
        if (elems[i] > threshold) {
            pat |= 1u;
            wide_count++;
        }
    }

    if (wide_count != BARCODE_WIDE_PER_CHAR) {
        return 0u; /* not a clean 3-wide Code 39 character */
    }
    *pattern9 = (uint16_t)(pat & 0x1FFu);
    return 1u;
}

/* ------------------------------------------------------------------ */
/* Frame decode                                                        */
/* ------------------------------------------------------------------ */
/*
 * Frame layout (29 elements):
 *   char0 (9) | gap (1) | char1 (9) | gap (1) | char2 (9) = 29.
 *   We decode char0, char1, char2 from fixed offsets 0, 10, 20 and
 *   ignore the inter-character gaps at 9 and 19.
 *   Valid iff char0 == '*' and char2 == '*' and char1 is a letter A..Z.
 *   Because '*' is asymmetric (reversed it reads 'P'), at most ONE scan
 *   direction can pass the guard check, so a reversed scan always yields
 *   the same letter as a forward scan.
 */
static barcode_t decode_oriented(const uint32_t w[BARCODE_FRAME_ELEMENTS])
{
    uint16_t p0, p1, p2;
    char c0, c1, c2;

    if (!barcode_classify_char(&w[0],  &p0)) {
        return barcode_invalid();
    }
    if (!barcode_classify_char(&w[10], &p1)) {
        return barcode_invalid();
    }
    if (!barcode_classify_char(&w[20], &p2)) {
        return barcode_invalid();
    }

    c0 = barcode_pattern_to_char(p0);
    c1 = barcode_pattern_to_char(p1);
    c2 = barcode_pattern_to_char(p2);

    if (c0 != '*' || c2 != '*') {
        return barcode_invalid();
    }
    if (c1 < 'A' || c1 > 'Z') {
        return barcode_invalid();
    }

    return barcode_pack_letter((uint8_t)(c1 - 'A'));
}

barcode_t barcode_decode_frame(const uint32_t widths[BARCODE_FRAME_ELEMENTS])
{
    barcode_t r = decode_oriented(widths);
    if (barcode_is_valid(r)) {
        return r;
    }

    /* Retry reversed: the car may read the barcode from the other side. */
    {
        uint32_t rev[BARCODE_FRAME_ELEMENTS];
        uint32_t i;
        for (i = 0u; i < BARCODE_FRAME_ELEMENTS; i++) {
            rev[i] = widths[BARCODE_FRAME_ELEMENTS - 1u - i];
        }
        r = decode_oriented(rev);
        if (barcode_is_valid(r)) {
            return r;
        }
    }
    return barcode_invalid();
}

uint32_t barcode_edges_to_widths(const barcode_edge_t *edges, uint32_t n_edges,
                                 uint32_t widths_out[BARCODE_FRAME_ELEMENTS])
{
    uint32_t i, n;
    if (n_edges < 2u) {
        return 0u;
    }
    n = n_edges - 1u;
    if (n > BARCODE_FRAME_ELEMENTS) {
        n = BARCODE_FRAME_ELEMENTS;
    }
    for (i = 0u; i < n; i++) {
        widths_out[i] = edges[i + 1u].t_us - edges[i].t_us;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* Pure frame assembler                                                */
/* ------------------------------------------------------------------ */
void barcode_asm_init(barcode_asm_t *a)
{
    a->count = 0u;
    a->last_edge_us = 0u;
    a->have_edge = 0u;
    a->quiet_timeout_us = (uint32_t)BARCODE_QUIET_TIMEOUT_MS * 1000u;
}

uint8_t barcode_asm_edge_arrived(barcode_asm_t *a, uint32_t t_us, barcode_t *out)
{
    barcode_edge_t edges[BARCODE_FRAME_EDGES];
    uint32_t widths[BARCODE_FRAME_ELEMENTS];
    uint32_t i;

    a->edges[a->count] = t_us;
    a->count++;
    a->last_edge_us = t_us;
    a->have_edge = 1u;

    if (a->count < BARCODE_FRAME_EDGES) {
        return 0u; /* frame not complete yet */
    }

    /* 30th edge: assemble and decode. */
    for (i = 0u; i < BARCODE_FRAME_EDGES; i++) {
        edges[i].t_us = a->edges[i];
    }
    (void)barcode_edges_to_widths(edges, BARCODE_FRAME_EDGES, widths);
    *out = barcode_decode_frame(widths);

    a->count = 0u;
    a->have_edge = 0u;
    return 1u;
}

uint8_t barcode_asm_time_passed(barcode_asm_t *a, uint32_t now_us, barcode_t *out)
{
    if (!a->have_edge || a->count == 0u) {
        return 0u; /* nothing partial to abandon */
    }
    if ((now_us - a->last_edge_us) > a->quiet_timeout_us) {
        *out = barcode_invalid(); /* abandon stalled partial frame, once */
        a->count = 0u;
        a->have_edge = 0u;
        return 1u;
    }
    return 0u;
}

/* ------------------------------------------------------------------ */
/* Edge queue (SPSC ring buffer)                                       */
/* ------------------------------------------------------------------ */
void barcode_eq_init(barcode_edge_queue_t *q)
{
    q->head = 0u;
    q->tail = 0u;
    q->dropped = 0u;
}

uint8_t barcode_eq_push(barcode_edge_queue_t *q, uint32_t t_us)
{
    uint8_t next = (uint8_t)((q->head + 1u) % BARCODE_EDGE_QUEUE_LEN);
    if (next == q->tail) {
        q->dropped++;        /* ISR is the sole writer; G3 counter */
        return 0u;
    }
    q->buf[q->head].t_us = t_us;
    q->head = next;
    return 1u;
}

uint8_t barcode_eq_pop(barcode_edge_queue_t *q, barcode_edge_t *out)
{
    if (q->tail == q->head) {
        return 0u;           /* empty */
    }
    *out = q->buf[q->tail];
    q->tail = (uint8_t)((q->tail + 1u) % BARCODE_EDGE_QUEUE_LEN);
    return 1u;
}

uint32_t barcode_eq_count(const barcode_edge_queue_t *q)
{
    return (uint32_t)(((uint32_t)q->head + BARCODE_EDGE_QUEUE_LEN - q->tail)
                      % BARCODE_EDGE_QUEUE_LEN);
}

uint8_t barcode_eq_overflow_delta(const barcode_edge_queue_t *q,
                                  uint8_t *last_seen)
{
    uint8_t now = q->dropped;            /* single atomic read of ISR counter */
    uint8_t delta = (uint8_t)(now - *last_seen); /* modular: handles wrap */
    *last_seen = now;                    /* task writes ONLY its own copy */
    return delta;
}

/* ------------------------------------------------------------------ */
/* Target-only ISR + task                                              */
/* ------------------------------------------------------------------ */
#ifndef BARCODE_HOST_TEST
#include "rtos_port.h"

/* Provided by the integration/board layer. */
extern barcode_edge_queue_t g_barcode_eq;
extern rtos_queue_t g_barcode_q;
extern rtos_flag_t  g_hb_flag;
extern uint32_t     g_barcode_hb_bit;

/* Board hook: current microsecond timestamp (e.g. time_us_32()). */
extern uint32_t board_now_us(void);

/* Right-IR GPIO edge callback: timestamp only, no interpretation. */
void barcode_edge_isr(void)
{
    (void)barcode_eq_push(&g_barcode_eq, board_now_us());
}

/*
 * Decode task: a thin loop around the pure assembler.
 *   - ISR stays timestamp-only and is the SOLE writer of q->dropped;
 *     this task is the SOLE writer of barcode_q.
 *   - barcode_asm_edge_arrived() decodes on the 30th edge.
 *   - barcode_asm_time_passed() abandons a stalled partial frame once.
 *   - Per G3, a queue overflow is handled as a COUNTER, not a fault bit:
 *     if any edges were dropped since the previous frame, that frame is
 *     marked INVALID and the drop is counted. The task detects this via
 *     its own last-seen copy (barcode_eq_overflow_delta), never writing
 *     q->dropped, so there is no two-writer race (Change Requirement 1).
 *   - The 20 ms poll keeps the heartbeat bit set even when idle.
 */
void barcode_decode_task(void)
{
    barcode_asm_t asm_state;
    uint8_t       seen_dropped = 0u; /* task-private last-seen drop count */
    barcode_asm_init(&asm_state);

    /* sync the baseline so pre-existing drops don't falsely taint frame 1 */
    (void)barcode_eq_overflow_delta(&g_barcode_eq, &seen_dropped);

    for (;;) {
        barcode_edge_t e;
        barcode_t      result;

        if (barcode_eq_pop(&g_barcode_eq, &e)) {
            if (barcode_asm_edge_arrived(&asm_state, e.t_us, &result)) {
                if (barcode_eq_overflow_delta(&g_barcode_eq, &seen_dropped) != 0u) {
                    result = barcode_invalid(); /* dropped edges -> untrusted */
                }
                (void)rtos_queue_send(g_barcode_q, &result, sizeof(result));
            }
        } else {
            (void)rtos_flag_set(g_hb_flag, g_barcode_hb_bit);

            if (barcode_asm_time_passed(&asm_state, board_now_us(), &result)) {
                (void)barcode_eq_overflow_delta(&g_barcode_eq, &seen_dropped);
                (void)rtos_queue_send(g_barcode_q, &result, sizeof(result));
            }
            rtos_delay_ms(BARCODE_FRAME_TIMEOUT_MS);
        }
    }
}
#endif /* BARCODE_HOST_TEST */

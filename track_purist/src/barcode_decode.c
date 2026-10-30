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
/* a >= 1.5 x b, without overflow (widths can be up to ~4e9 us). */
static uint8_t at_least_split_ratio(uint32_t a, uint32_t b)
{
    return ((uint64_t)a * BARCODE_SPLIT_RATIO_DEN >=
            (uint64_t)b * BARCODE_SPLIT_RATIO_NUM) ? 1u : 0u;
}

/*
 * Relative split, no absolute widths: the 3 widest of the 9 elements are
 * wide (Code 39 always has exactly 3). The split must be clean: the
 * narrowest wide element >= 1.5 x the widest narrow element. Ties and
 * near-ties fail, so an ambiguous character is never guessed.
 * On success also returns the narrowest wide width (for the gap guard).
 */
static uint8_t classify_split(const uint32_t elems[BARCODE_CHAR_ELEMENTS],
                              uint16_t *pattern9, uint32_t *min_wide)
{
    uint8_t  is_wide[BARCODE_CHAR_ELEMENTS] = { 0u };
    uint32_t w_min = 0xFFFFFFFFu;
    uint32_t n_max = 0u;
    uint16_t pat = 0u;
    uint8_t  k, i;

    for (k = 0u; k < BARCODE_WIDE_PER_CHAR; k++) {
        uint8_t best = BARCODE_CHAR_ELEMENTS;
        for (i = 0u; i < BARCODE_CHAR_ELEMENTS; i++) {
            if (!is_wide[i] &&
                (best == BARCODE_CHAR_ELEMENTS || elems[i] > elems[best])) {
                best = i;
            }
        }
        is_wide[best] = 1u;
        if (elems[best] < w_min) {
            w_min = elems[best];
        }
    }
    for (i = 0u; i < BARCODE_CHAR_ELEMENTS; i++) {
        pat = (uint16_t)(pat << 1);
        if (is_wide[i]) {
            pat |= 1u;
        } else if (elems[i] > n_max) {
            n_max = elems[i];
        }
    }

    if (!at_least_split_ratio(w_min, n_max)) {
        return 0u; /* 3rd and 4th widest too close: ambiguous */
    }
    *pattern9 = (uint16_t)(pat & 0x1FFu);
    *min_wide = w_min;
    return 1u;
}

uint8_t barcode_classify_char(const uint32_t elems[BARCODE_CHAR_ELEMENTS],
                              uint16_t *pattern9)
{
    uint32_t w_min;
    return classify_split(elems, pattern9, &w_min);
}

/* ------------------------------------------------------------------ */
/* Frame decode                                                        */
/* ------------------------------------------------------------------ */
/*
 * Frame layout (29 elements):
 *   char0 (9) | gap (1) | char1 (9) | gap (1) | char2 (9) = 29.
 *   char0, char1, char2 sit at fixed offsets 0, 10, 20.
 *   Valid iff char0 == '*' and char2 == '*', char1 is a letter A..Z, and
 *   both gaps (offsets 9 and 19) are narrow: the narrowest wide element
 *   of each neighbouring character is >= 1.5 x the gap. The gap guard
 *   rejects windows that are shifted by stray or lost edges.
 *   Because '*' is asymmetric (reversed it reads 'P'), at most ONE scan
 *   direction can pass the guard check, so a reversed scan always yields
 *   the same letter as a forward scan.
 */
static barcode_t decode_oriented(const uint32_t w[BARCODE_FRAME_ELEMENTS])
{
    uint16_t p0, p1, p2;
    uint32_t m0, m1, m2;
    char c0, c1, c2;

    if (!classify_split(&w[0],  &p0, &m0) ||
        !classify_split(&w[10], &p1, &m1) ||
        !classify_split(&w[20], &p2, &m2)) {
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
    /* gap guard: both inter-character gaps must be narrow */
    if (!at_least_split_ratio(m0, w[9])  || !at_least_split_ratio(m1, w[9]) ||
        !at_least_split_ratio(m1, w[19]) || !at_least_split_ratio(m2, w[19])) {
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
/*
 * Invariant: two accepted edges are always >= BARCODE_CHATTER_US apart.
 * An edge is accepted only when the NEXT edge arrives >= 2 ms later or
 * 2 ms pass with no edge; an edge < 2 ms after a pending one cancels both.
 * Bounce at a real transition (odd number of edges) leaves one edge; a
 * glitch inside a bar (even number) leaves none, so bar/space order holds.
 */
void barcode_asm_init(barcode_asm_t *a)
{
    a->pending_us         = 0u;
    a->last_edge_us       = 0u;
    a->burst_edges        = 0u;
    a->chatter_pairs      = 0u;
    a->last_burst_edges   = 0u;
    a->last_chatter_pairs = 0u;
    a->count              = 0u;
    a->have_pending       = 0u;
    a->last_count         = 0u;
    a->last_outcome       = (uint8_t)BARCODE_OUT_NONE;
}

/* Freeze the debug counters for the result just produced, clear window. */
static void asm_finish(barcode_asm_t *a, barcode_outcome_t outcome)
{
    a->last_count         = a->count;
    a->last_outcome       = (uint8_t)outcome;
    a->last_burst_edges   = a->burst_edges;
    a->last_chatter_pairs = a->chatter_pairs;
    a->count              = 0u;   /* edges[] kept for barcode_asm_last_frame */
    a->burst_edges        = 0u;
    a->chatter_pairs      = 0u;
}

/* Accept one edge into the rolling window; decode once it holds 30. */
static uint8_t asm_accept(barcode_asm_t *a, uint32_t t_us, barcode_t *out)
{
    uint32_t widths[BARCODE_FRAME_ELEMENTS];
    barcode_edge_t edges[BARCODE_FRAME_EDGES];
    barcode_t r;
    uint32_t i;

    if (a->count == BARCODE_FRAME_EDGES) {
        /* window full: the oldest edge slides out (stray before a barcode) */
        for (i = 0u; i + 1u < BARCODE_FRAME_EDGES; i++) {
            a->edges[i] = a->edges[i + 1u];
        }
        a->count--;
    }
    a->edges[a->count] = t_us;
    a->count++;
    if (a->burst_edges < 0xFFFFu) {
        a->burst_edges++;
    }
    if (a->count < BARCODE_FRAME_EDGES) {
        return 0u;
    }

    for (i = 0u; i < BARCODE_FRAME_EDGES; i++) {
        edges[i].t_us = a->edges[i];
    }
    (void)barcode_edges_to_widths(edges, BARCODE_FRAME_EDGES, widths);
    r = barcode_decode_frame(widths);
    if (!barcode_is_valid(r)) {
        return 0u; /* not reported: the next edge may complete a good window */
    }
    asm_finish(a, BARCODE_OUT_VALID);
    *out = r;
    return 1u;
}

uint32_t barcode_asm_abort_timeout_us(const barcode_asm_t *a)
{
    uint32_t longest = 0u;
    uint32_t i;

    if (a->count < 2u) {
        return BARCODE_ABORT_CAP_US; /* no element measured yet */
    }
    for (i = 0u; i + 1u < a->count; i++) {
        uint32_t w = a->edges[i + 1u] - a->edges[i];
        if (w > longest) {
            longest = w;
        }
    }
    if (longest >= BARCODE_ABORT_CAP_US / BARCODE_ABORT_FACTOR) {
        return BARCODE_ABORT_CAP_US;
    }
    longest *= BARCODE_ABORT_FACTOR;
    return (longest < BARCODE_ABORT_FLOOR_US) ? BARCODE_ABORT_FLOOR_US : longest;
}

uint8_t barcode_asm_edge_arrived(barcode_asm_t *a, uint32_t t_us, barcode_t *out)
{
    uint8_t produced = 0u;

    a->last_edge_us = t_us;
    if (a->have_pending) {
        if ((t_us - a->pending_us) < BARCODE_CHATTER_US) {
            /* glitch pulse: drop BOTH edges, bar/space order is kept */
            a->have_pending = 0u;
            if (a->chatter_pairs < 0xFFFFu) {
                a->chatter_pairs++;
            }
            return 0u;
        }
        produced = asm_accept(a, a->pending_us, out);
    }
    a->pending_us   = t_us;
    a->have_pending = 1u;
    return produced;
}

uint8_t barcode_asm_time_passed(barcode_asm_t *a, uint32_t now_us, barcode_t *out)
{
    if (a->have_pending) {
        if ((now_us - a->pending_us) < BARCODE_CHATTER_US) {
            return 0u; /* chatter check still running */
        }
        a->have_pending = 0u;
        if (asm_accept(a, a->pending_us, out)) {
            return 1u;
        }
    }
    if (a->count == 0u) {
        return 0u;
    }
    if ((now_us - a->last_edge_us) <= barcode_asm_abort_timeout_us(a)) {
        return 0u;
    }
    asm_finish(a, (a->burst_edges >= BARCODE_FRAME_EDGES) ?
                  BARCODE_OUT_UNDECODABLE : BARCODE_OUT_PARTIAL);
    *out = barcode_invalid(); /* abandoned, reported once */
    return 1u;
}

void barcode_asm_last_frame(const barcode_asm_t *a, barcode_frame_info_t *p_info)
{
    uint32_t i;

    p_info->n_edges       = a->last_count;
    p_info->n_widths      = (a->last_count > 1u) ? (uint8_t)(a->last_count - 1u) : 0u;
    p_info->outcome       = a->last_outcome;
    p_info->burst_edges   = a->last_burst_edges;
    p_info->chatter_pairs = a->last_chatter_pairs;
    p_info->longest_us    = 0u;
    for (i = 0u; i < p_info->n_widths; i++) {
        p_info->widths_us[i] = a->edges[i + 1u] - a->edges[i];
        if (p_info->widths_us[i] > p_info->longest_us) {
            p_info->longest_us = p_info->widths_us[i];
        }
    }
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
 *   - A result can come from either assembler call (an edge is accepted
 *     only after its 2 ms chatter check), so both go through one path.
 *   - Per G3, a queue overflow is handled as a COUNTER, not a fault bit:
 *     if any edges were dropped since the previous result, that result is
 *     forced INVALID. The task detects this via its own last-seen copy
 *     (barcode_eq_overflow_delta), never writing q->dropped, so there is
 *     no two-writer race (Change Requirement 1).
 *   - The 20 ms poll keeps the heartbeat bit set even when idle.
 */
static void publish_result(barcode_t result, uint8_t *p_seen_dropped)
{
    if (barcode_eq_overflow_delta(&g_barcode_eq, p_seen_dropped) != 0u) {
        result = barcode_invalid(); /* dropped edges -> untrusted */
    }
    (void)rtos_queue_send(g_barcode_q, &result, sizeof(result));
}

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
                publish_result(result, &seen_dropped);
            }
        } else {
            (void)rtos_flag_set(g_hb_flag, g_barcode_hb_bit);

            if (barcode_asm_time_passed(&asm_state, board_now_us(), &result)) {
                publish_result(result, &seen_dropped);
            }
            rtos_delay_ms(BARCODE_FRAME_TIMEOUT_MS);
        }
    }
}
#endif /* BARCODE_HOST_TEST */

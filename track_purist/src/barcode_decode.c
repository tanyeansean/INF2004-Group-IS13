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
 * Relative split, no absolute widths. 20 Nov (Aurelius): bars are compared
 * only with bars and spaces only with spaces.
 *
 * Before, the 3 widest of all 9 elements were "wide" and the narrowest of
 * them had to be >= 1.5 x the widest narrow one. A real IR sensor sees
 * black a little early (or late), so every bar reads longer and every
 * space shorter by the same amount g. The narrowest wide element is then
 * the wide SPACE (3u - g) and the widest narrow one a narrow BAR (u + g):
 * the rule needed g <= 0.6 u. That is 1.8 mm on the 141 mm barcode
 * (u = 3 mm) but only 1.2 mm at u = 2 mm and 0.9 mm at u = 1.5 mm, so the
 * same sensor read the big barcodes and failed the small ones with all 30
 * edges present (PC model: testing/subsystem3/pc/barsim_small.c).
 *
 * Every letter A..Z and '*' has exactly 2 wide bars (of 5) and 1 wide
 * space (of 4), so: the 2 widest bars and the widest space are wide, and
 * each group must split cleanly on its own (>= 1.5 x). g cancels inside a
 * group. Anything the old rule accepted is still accepted; ties and
 * near-ties still fail, so an ambiguous character is never guessed.
 * On success also returns the wide space's width for the gap guard (the
 * gaps are spaces, so that is the like-for-like comparison).
 */
static uint8_t classify_split(const uint32_t elems[BARCODE_CHAR_ELEMENTS],
                              uint16_t *pattern9, uint32_t *min_wide)
{
    uint8_t  is_wide[BARCODE_CHAR_ELEMENTS] = { 0u };
    uint32_t wide_bar_min = 0xFFFFFFFFu;
    uint32_t narrow_bar_max = 0u;
    uint32_t wide_space;
    uint32_t narrow_space_max = 0u;
    uint16_t pat = 0u;
    uint8_t  k, i, best;

    /* bars: elements 0, 2, 4, 6, 8 - the 2 widest are wide */
    for (k = 0u; k < 2u; k++) {
        best = BARCODE_CHAR_ELEMENTS;
        for (i = 0u; i < BARCODE_CHAR_ELEMENTS; i += 2u) {
            if (!is_wide[i] &&
                (best == BARCODE_CHAR_ELEMENTS || elems[i] > elems[best])) {
                best = i;
            }
        }
        is_wide[best] = 1u;
        if (elems[best] < wide_bar_min) {
            wide_bar_min = elems[best];
        }
    }
    /* spaces: elements 1, 3, 5, 7 - the widest is wide */
    best = 1u;
    for (i = 3u; i < BARCODE_CHAR_ELEMENTS; i += 2u) {
        if (elems[i] > elems[best]) {
            best = i;
        }
    }
    is_wide[best] = 1u;
    wide_space = elems[best];

    for (i = 0u; i < BARCODE_CHAR_ELEMENTS; i++) {
        pat = (uint16_t)(pat << 1);
        if (is_wide[i]) {
            pat |= 1u;
        } else if ((i & 1u) == 0u) {
            if (elems[i] > narrow_bar_max) {
                narrow_bar_max = elems[i];
            }
        } else {
            if (elems[i] > narrow_space_max) {
                narrow_space_max = elems[i];
            }
        }
    }

    if (!at_least_split_ratio(wide_bar_min, narrow_bar_max) ||
        !at_least_split_ratio(wide_space, narrow_space_max)) {
        return 0u; /* a group does not split cleanly: ambiguous */
    }
    *pattern9 = (uint16_t)(pat & 0x1FFu);
    *min_wide = wide_space;
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
    a->last_repair        = BARCODE_REPAIR_NONE;
}

/* Freeze the debug counters for the result just produced, clear window. */
/* ------------------------------------------------------------------ */
/* Repairs (20 Nov, Aurelius, from the lab log of that day)            */
/* ------------------------------------------------------------------ */
/*
 * Both repairs only ever hand a full 29-element frame to
 * barcode_decode_frame(), so every normal check still applies to it: both
 * '*', a letter, clean bar and space splits, both gap guards.
 *
 * 1. Speck. A scuff or dirt mark inside a white space (or a pinhole in a
 *    bar) adds 2 edges: 32 instead of 30, and no 30-edge window decodes.
 *    Lab, barcode Z, 12:11 and 12:27: "... 27 34 [3 2] 30 ..." where the
 *    clean passes read "... 28 37 28 ...". With all 32 edges in view, each
 *    inner element that is no more than 0.4 x the narrowest element left
 *    after removing it is merged into its two neighbours and the 29
 *    widths decoded. Accepted only if every candidate that decodes gives
 *    the same letter.
 *
 * 2. Cut-off end mark. If the paper has no white margin after the last
 *    bar (bars to the paper edge, then mat or a printed line), the last
 *    bar never ends and the pass goes quiet 1 to 3 elements short. Lab,
 *    barcode A, 10:06, 10:13, 10:31. The first and last character are the
 *    same '*' read the same way round, so the missing elements of one are
 *    copied from the other and the frame decoded. At least 6 of that
 *    mark's 9 elements are still measured, including one wide space or
 *    two wide bars, and must fit. Tried for a short tail and a short
 *    head; accepted only if the results agree.
 */
#if BARCODE_REPAIR_SPECK
static barcode_t repair_speck(const uint32_t e[BARCODE_WIN_EDGES])
{
    uint32_t w[BARCODE_WIN_EDGES - 1u];
    uint32_t m[BARCODE_FRAME_ELEMENTS];
    barcode_t found = barcode_invalid();
    uint32_t i, j, k, least;

    for (i = 0u; i + 1u < BARCODE_WIN_EDGES; i++) {
        w[i] = e[i + 1u] - e[i];
    }
    for (i = 1u; i + 2u < BARCODE_WIN_EDGES; i++) {   /* inner elements 1..29 */
        barcode_t r;

        k = 0u;
        least = 0xFFFFFFFFu;
        for (j = 0u; j + 1u < BARCODE_WIN_EDGES; j++) {
            if (j == i - 1u) {
                m[k] = w[i - 1u] + w[i] + w[i + 1u];
                j = i + 1u;
            } else {
                m[k] = w[j];
            }
            if (m[k] < least) {
                least = m[k];
            }
            k++;
        }
        if (((uint64_t)w[i] * 5u) > ((uint64_t)least * 2u)) {
            continue;   /* not small enough to be a speck */
        }
        r = barcode_decode_frame(m);
        if (!barcode_is_valid(r)) {
            continue;
        }
        if (barcode_is_valid(found) && (found.packed != r.packed)) {
            return barcode_invalid();   /* two readings: do not guess */
        }
        found = r;
    }
    return found;
}
#endif

#if BARCODE_REPAIR_TRUNC
static barcode_t repair_truncated(const uint32_t *e, uint32_t n_edges)
{
    uint32_t f[BARCODE_FRAME_ELEMENTS];
    uint32_t const n = n_edges - 1u;                    /* measured widths */
    uint32_t const missing = BARCODE_FRAME_ELEMENTS - n;
    barcode_t tail, head;
    uint32_t i;

    /* short tail: measured widths are elements 0..n-1 */
    for (i = 0u; i < n; i++) {
        f[i] = e[i + 1u] - e[i];
    }
    for (i = n; i < BARCODE_FRAME_ELEMENTS; i++) {
        f[i] = f[i - 20u];              /* same element of the first '*' */
    }
    tail = barcode_decode_frame(f);

    /* short head: measured widths are elements missing..28 */
    for (i = 0u; i < n; i++) {
        f[missing + i] = e[i + 1u] - e[i];
    }
    for (i = 0u; i < missing; i++) {
        f[i] = f[i + 20u];              /* same element of the last '*' */
    }
    head = barcode_decode_frame(f);

    if (barcode_is_valid(tail) && barcode_is_valid(head)) {
        return (tail.packed == head.packed) ? tail : barcode_invalid();
    }
    return barcode_is_valid(tail) ? tail : head;
}
#endif

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

    a->last_repair = BARCODE_REPAIR_NONE;
    if (a->count == BARCODE_WIN_EDGES) {
        /* window full: the oldest edge slides out (stray before a barcode) */
        for (i = 0u; i + 1u < BARCODE_WIN_EDGES; i++) {
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

    /* the newest 30 edges, as before the window grew to 32 */
    for (i = 0u; i < BARCODE_FRAME_EDGES; i++) {
        edges[i].t_us = a->edges[(a->count - BARCODE_FRAME_EDGES) + i];
    }
    (void)barcode_edges_to_widths(edges, BARCODE_FRAME_EDGES, widths);
    r = barcode_decode_frame(widths);
#if BARCODE_REPAIR_SPECK
    if (!barcode_is_valid(r) && (a->count == BARCODE_WIN_EDGES)) {
        r = repair_speck(a->edges);
        if (barcode_is_valid(r)) {
            a->last_repair = BARCODE_REPAIR_SPECK_USED;
        }
    }
#endif
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
#if BARCODE_REPAIR_TRUNC
    if ((a->count < BARCODE_FRAME_EDGES) &&
        ((a->count + BARCODE_TRUNC_MAX_MISSING) >= BARCODE_FRAME_EDGES) &&
        (a->burst_edges == a->count)) {
        barcode_t const r = repair_truncated(a->edges, a->count);

        if (barcode_is_valid(r)) {
            a->last_repair = BARCODE_REPAIR_TRUNC_USED;
            asm_finish(a, BARCODE_OUT_VALID);
            *out = r;
            return 1u;
        }
    }
#endif
    asm_finish(a, (a->burst_edges >= BARCODE_FRAME_EDGES) ?
                  BARCODE_OUT_UNDECODABLE : BARCODE_OUT_PARTIAL);
    *out = barcode_invalid(); /* abandoned, reported once */
    return 1u;
}

void barcode_asm_last_frame(const barcode_asm_t *a, barcode_frame_info_t *p_info)
{
    uint32_t i;

    /* the newest 30 edges of the window (it can hold 32) */
    uint32_t const skip = (a->last_count > BARCODE_FRAME_EDGES) ?
                          ((uint32_t)a->last_count - BARCODE_FRAME_EDGES) : 0u;

    p_info->n_edges       = (uint8_t)(a->last_count - skip);
    p_info->n_widths      = (p_info->n_edges > 1u) ? (uint8_t)(p_info->n_edges - 1u) : 0u;
    p_info->outcome       = a->last_outcome;
    p_info->burst_edges   = a->last_burst_edges;
    p_info->chatter_pairs = a->last_chatter_pairs;
    p_info->longest_us    = 0u;
    for (i = 0u; i < p_info->n_widths; i++) {
        p_info->widths_us[i] = a->edges[skip + i + 1u] - a->edges[skip + i];
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

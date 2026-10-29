/**
 * @file test_barcode.c
 * @brief Host unit tests for the barcode decode core and edge queue.
 *
 * Builds synthetic 29-element frames from Code 39 patterns and asserts
 * decode results. Narrow element = 3 units, wide element = 9 units
 * (the 3 mm / 9 mm bars from the spec), so the midpoint classifier has
 * a clear gap to work with.
 */
#ifndef BARCODE_HOST_TEST
#define BARCODE_HOST_TEST  /* also set by the test CMake target */
#endif
#include "barcode_decode.h"
#include "test_util.h"
#include <string.h>

#define NARROW 3u
#define WIDE   9u

/* Pattern constants for test construction. Letters come from the
   decoder's own table (barcode_letter_pattern) and are independently
   checked below against the course samples and Code 39 rules. */
#define T_STAR 0x094u
#define T_A    0x109u   /* course sample A */
#define T_B    0x049u
#define T_C    0x148u
#define T_D    0x019u
#define T_F    0x058u   /* course sample F */
#define T_W    0x1C0u   /* course sample W */
#define T_X    0x091u   /* course sample X */
#define T_Y    0x190u   /* course sample Y */
#define T_Z    0x0D0u   /* course sample Z */
#define T_P    0x052u   /* '*' read backwards */
#define T_DIGIT0 0x034u /* Code 39 '0' (not a letter -> INVALID) */

#define L(ch)  ((uint8_t)((ch) - 'A'))

/* Reverse the 9 element order of a pattern (what a backwards scan sees). */
static uint16_t reverse9(uint16_t p)
{
    uint16_t r = 0u;
    int i;
    for (i = 0; i < 9; i++) {
        r = (uint16_t)((r << 1) | ((p >> i) & 1u));
    }
    return r;
}

/* Expand a 9-bit pattern (MSB first) into 9 element widths. */
static void emit_char(uint16_t pat, uint32_t *out)
{
    int i;
    for (i = 0; i < 9; i++) {
        uint16_t bit = (uint16_t)((pat >> (8 - i)) & 1u);
        out[i] = bit ? WIDE : NARROW;
    }
}

/*
 * Build a 29-element frame: char0(9) gap(1) char1(9) gap(1) char2(9).
 * Gaps are narrow inter-character spaces; their exact width is
 * ignored by the decoder (it reads offsets 0, 10, 20).
 */
static void build_frame(uint16_t p0, uint16_t p1, uint16_t p2,
                        uint32_t frame[BARCODE_FRAME_ELEMENTS])
{
    uint32_t i;
    for (i = 0; i < BARCODE_FRAME_ELEMENTS; i++) frame[i] = NARROW;
    emit_char(p0, &frame[0]);
    emit_char(p1, &frame[10]);
    emit_char(p2, &frame[20]);
}

static void reverse_frame(const uint32_t in[BARCODE_FRAME_ELEMENTS],
                          uint32_t out[BARCODE_FRAME_ELEMENTS])
{
    uint32_t i;
    for (i = 0; i < BARCODE_FRAME_ELEMENTS; i++) {
        out[i] = in[BARCODE_FRAME_ELEMENTS - 1 - i];
    }
}

/*
 * Convert a 29-element width frame into 30 edge timestamps starting at
 * t0, with each element scaled by us_per_unit microseconds. NARROW(3)
 * and WIDE(9) units map to proportional durations.
 */
static void frame_to_edges(const uint32_t frame[BARCODE_FRAME_ELEMENTS],
                           uint32_t t0, uint32_t us_per_unit,
                           uint32_t edges_out[BARCODE_FRAME_EDGES])
{
    uint32_t i, t = t0;
    edges_out[0] = t;
    for (i = 0; i < BARCODE_FRAME_ELEMENTS; i++) {
        t += frame[i] * us_per_unit;
        edges_out[i + 1] = t;
    }
}

/*
 * Drive a sequence of edge timestamps through the assembler, counting
 * VALID and INVALID results. After the last edge, time advances by the
 * chatter window so the last edge is accepted (as the poll loop would).
 * Returns the last valid letter seen.
 */
static void run_edges(barcode_asm_t *a,
                      const uint32_t *edges, uint32_t n,
                      int *valid_count, int *invalid_count,
                      uint8_t *last_letter)
{
    uint32_t i;
    barcode_t out;
    for (i = 0; i < n; i++) {
        if (barcode_asm_edge_arrived(a, edges[i], &out)) {
            if (barcode_is_valid(out)) {
                (*valid_count)++;
                *last_letter = barcode_get_letter(out);
            } else {
                (*invalid_count)++;
            }
        }
    }
    if (n > 0u && barcode_asm_time_passed(a, edges[n - 1u] + BARCODE_CHATTER_US, &out)) {
        if (barcode_is_valid(out)) {
            (*valid_count)++;
            *last_letter = barcode_get_letter(out);
        } else {
            (*invalid_count)++;
        }
    }
}

/* ================================================================== */
/* Physical-scan simulation: print size, speed, chatter, stray edges   */
/* ================================================================== */

/* Deterministic PRNG (same numbers on every machine). */
static uint32_t g_rng = 12345u;
static uint32_t rng_next(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng >> 8;
}
static uint32_t rng_range(uint32_t lo, uint32_t hi)
{
    return lo + rng_next() % (hi - lo + 1u);
}

#define MAX_STREAM 512u
typedef struct {
    uint32_t t[MAX_STREAM];
    uint32_t n;
    uint32_t pairs;      /* glitch pulses / bounces injected (expected cancels) */
} stream_t;

static void push(stream_t *s, uint32_t t)
{
    if (s->n < MAX_STREAM) {
        s->t[s->n++] = t;
    }
}

/*
 * How the barcode passes the sensor. Spec size: 1 unit = 1 mm, so the
 * narrow bar is 3 mm and the wide bar 9 mm at scale 1.0 (A4 long side
 * = 297 mm = scale 2.1). Speed v_mm_s for element 0, then:
 *   ramp   : speed changes linearly to v x ramp at the last element
 *   wobble : +/- wobble x v, triangle wave with a period of one character
 *   step_at/step_factor: from element step_at on, speed x step_factor
 */
typedef struct {
    double  scale;
    double  v_mm_s;
    double  ramp;
    double  wobble;
    uint8_t step_at;
    double  step_factor;
} scan_t;

static scan_t scan(double scale, double v)
{
    scan_t sc = { scale, v, 1.0, 0.0, 255u, 1.0 };
    return sc;
}

static double elem_speed(const scan_t *sc, uint32_t i)
{
    double v = sc->v_mm_s * (1.0 + (sc->ramp - 1.0) * (double)i / 28.0);
    double ph = (double)(i % 9u) / 9.0;
    double tri = 4.0 * (ph > 0.5 ? ph - 0.5 : 0.5 - ph) - 1.0;   /* -1..1 */
    v *= 1.0 + sc->wobble * tri;
    if (i >= sc->step_at) {
        v *= sc->step_factor;
    }
    return v;
}

/* Element durations in us for one barcode (forward or backward scan). */
static void barcode_durations(uint8_t letter, uint8_t backward, const scan_t *sc,
                              uint32_t dur[BARCODE_FRAME_ELEMENTS])
{
    uint32_t f[BARCODE_FRAME_ELEMENTS], r[BARCODE_FRAME_ELEMENTS];
    const uint32_t *src = f;
    uint32_t i;
    build_frame(T_STAR, barcode_letter_pattern(letter), T_STAR, f);
    if (backward) {
        reverse_frame(f, r);
        src = r;
    }
    for (i = 0; i < BARCODE_FRAME_ELEMENTS; i++) {
        double mm = (double)src[i] * sc->scale;
        dur[i] = (uint32_t)(mm / elem_speed(sc, i) * 1e6 + 0.5);
    }
}

/* Append the 30 edges of one barcode, first edge at t0. Returns end time. */
static uint32_t add_barcode(stream_t *s, uint8_t letter, uint8_t backward,
                            const scan_t *sc, uint32_t t0)
{
    uint32_t dur[BARCODE_FRAME_ELEMENTS];
    uint32_t i, t = t0;
    barcode_durations(letter, backward, sc, dur);
    push(s, t);
    for (i = 0; i < BARCODE_FRAME_ELEMENTS; i++) {
        t += dur[i];
        push(s, t);
    }
    return t;
}

typedef struct {
    int right;          /* VALID with the expected letter */
    int wrong;          /* VALID with any other letter    */
    int invalid;
    uint32_t pairs_seen;   /* chatter pairs cancelled in the VALID burst */
    uint8_t  edges_seen;   /* window size of the VALID result            */
} tally_t;

static void count_result(const barcode_asm_t *a, barcode_t out, uint8_t expect, tally_t *r)
{
    if (barcode_is_valid(out)) {
        barcode_frame_info_t info;
        barcode_asm_last_frame(a, &info);
        if (barcode_get_letter(out) == expect) {
            r->right++;
            r->pairs_seen = info.chatter_pairs;
            r->edges_seen = info.n_edges;
        } else {
            r->wrong++;
        }
    } else {
        r->invalid++;
    }
}

/*
 * Feed a stream through a fresh assembler. Between edges time_passed is
 * called every poll_us (0 = only just before the next edge, which is the
 * worst case for the abort timer), then 3 s of quiet at the end.
 */
static tally_t feed(const stream_t *s, uint8_t expect, uint32_t poll_us)
{
    barcode_asm_t a;
    barcode_t out;
    tally_t r = { 0, 0, 0, 0u, 0u };
    uint32_t i, t;
    barcode_asm_init(&a);
    for (i = 0; i < s->n; i++) {
        if (i > 0u) {
            if (poll_us != 0u) {
                for (t = s->t[i - 1] + poll_us; t < s->t[i]; t += poll_us) {
                    if (barcode_asm_time_passed(&a, t, &out)) count_result(&a, out, expect, &r);
                }
            } else if (s->t[i] - s->t[i - 1] > 1u) {
                if (barcode_asm_time_passed(&a, s->t[i] - 1u, &out)) count_result(&a, out, expect, &r);
            }
        }
        if (barcode_asm_edge_arrived(&a, s->t[i], &out)) count_result(&a, out, expect, &r);
    }
    if (s->n > 0u) {
        uint32_t last = s->t[s->n - 1u];
        uint32_t step = (poll_us != 0u) ? poll_us : 1000u;
        for (t = last + step; t <= last + 3000000u; t += step) {
            if (barcode_asm_time_passed(&a, t, &out)) count_result(&a, out, expect, &r);
        }
    }
    return r;
}

/* Insert one glitch pulse (2 edges, width 0.1..1.9 ms) inside element i. */
static uint32_t add_glitch_inside(stream_t *s, uint32_t start, uint32_t dur)
{
    uint32_t w, off;
    if (dur < 2400u) {
        return 0u;                               /* no room for a pulse */
    }
    w = rng_range(100u, (dur - 300u < 1900u) ? dur - 300u : 1900u);
    off = rng_range(100u, dur - w - 100u);
    push(s, start + off);
    push(s, start + off + w);
    return 1u;
}

int main(void)
{
    printf("== barcode decode tests ==\n");

    /* --- pattern table sanity: Code 39 rules for every letter --- */
    {
        uint8_t i, j;
        int ok_rules = 1, ok_unique = 1;
        for (i = 0; i < BARCODE_LETTER_COUNT; i++) {
            uint16_t p = barcode_letter_pattern(i);
            int bars = 0, spaces = 0, k;
            for (k = 0; k < 9; k++) {
                if ((p >> (8 - k)) & 1u) {
                    if (k % 2 == 0) bars++; else spaces++;
                }
            }
            if (bars != 2 || spaces != 1) ok_rules = 0;
            if (p == T_STAR) ok_unique = 0;
            for (j = (uint8_t)(i + 1u); j < BARCODE_LETTER_COUNT; j++) {
                if (barcode_letter_pattern(j) == p) ok_unique = 0;
            }
        }
        CHECK(ok_rules, "every letter A..Z: 2 wide bars + 1 wide space");
        CHECK(ok_unique, "every letter pattern unique and != '*'");
        CHECK_EQ_INT(barcode_letter_pattern(L('A')), T_A, "table A = course sample A");
        CHECK_EQ_INT(barcode_letter_pattern(L('F')), T_F, "table F = course sample F");
        CHECK_EQ_INT(barcode_letter_pattern(L('W')), T_W, "table W = course sample W");
        CHECK_EQ_INT(barcode_letter_pattern(L('X')), T_X, "table X = course sample X");
        CHECK_EQ_INT(barcode_letter_pattern(L('Y')), T_Y, "table Y = course sample Y");
        CHECK_EQ_INT(barcode_letter_pattern(L('Z')), T_Z, "table Z = course sample Z");
        CHECK_EQ_INT(reverse9(T_STAR), T_P, "'*' read backwards is 'P'");
        CHECK_EQ_INT(barcode_pattern_to_char(T_P), 'P', "so a backwards guard never reads as '*'");
    }

    /* --- every letter A..Z decodes, forwards AND backwards, same letter --- */
    {
        uint8_t i;
        int fwd_ok = 1, rev_ok = 1;
        for (i = 0; i < BARCODE_LETTER_COUNT; i++) {
            uint32_t f[BARCODE_FRAME_ELEMENTS], rev[BARCODE_FRAME_ELEMENTS];
            barcode_t r;
            build_frame(T_STAR, barcode_letter_pattern(i), T_STAR, f);
            r = barcode_decode_frame(f);
            if (!barcode_is_valid(r) || barcode_get_letter(r) != i) fwd_ok = 0;
            reverse_frame(f, rev);
            r = barcode_decode_frame(rev);
            if (!barcode_is_valid(r) || barcode_get_letter(r) != i) rev_ok = 0;
        }
        CHECK(fwd_ok, "all 26 letters decode correctly (forward scan)");
        CHECK(rev_ok, "all 26 letters decode to the SAME letter (backward scan)");
    }

    /* --- course samples individually --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS], rev[BARCODE_FRAME_ELEMENTS];
        barcode_t r;
        build_frame(T_STAR, T_A, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(barcode_is_valid(r), "*A* decodes valid");
        CHECK_EQ_INT(barcode_get_char(r), 'A', "*A* -> 'A'");

        build_frame(T_STAR, T_Z, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(barcode_is_valid(r), "*Z* decodes valid (A..Z all accepted)");
        CHECK_EQ_INT(barcode_get_char(r), 'Z', "*Z* -> 'Z'");

        build_frame(T_STAR, T_F, T_STAR, f);
        reverse_frame(f, rev);
        r = barcode_decode_frame(rev);
        CHECK_EQ_INT(barcode_get_char(r), 'F', "backward *F* -> 'F'");
    }

    /* --- different print scale / speed: only the wide:narrow ratio matters --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS];
        uint32_t i;
        barcode_t r;
        build_frame(T_STAR, T_W, T_STAR, f);
        for (i = 0; i < BARCODE_FRAME_ELEMENTS; i++) {
            f[i] = (f[i] == WIDE) ? 5u : 2u; /* 2.5:1 instead of 3:1 */
        }
        r = barcode_decode_frame(f);
        CHECK_EQ_INT(barcode_get_char(r), 'W', "W decodes at 2.5:1 ratio");
    }

    /* --- non-letters (digits) -> INVALID --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS];
        barcode_t r;
        build_frame(T_STAR, T_DIGIT0, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(!barcode_is_valid(r), "digit '0' -> INVALID (letters only)");
    }

    /* --- packing --- */
    {
        barcode_t b = barcode_pack_letter(L('Z'));
        CHECK(barcode_is_valid(b), "pack Z valid");
        CHECK_EQ_INT(barcode_get_letter(b), 25, "pack Z letter 25");
        CHECK_EQ_INT(b.packed & 0xC0u, 0, "bits 6..7 stay 0");
        b = barcode_pack_letter(26u);
        CHECK(!barcode_is_valid(b), "letter 26 -> INVALID");
        CHECK_EQ_INT(barcode_get_char(barcode_invalid()), '?', "invalid -> '?'");
    }

    /* --- missing start/stop guard -> INVALID --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS];
        barcode_t r;
        build_frame(T_A, T_B, T_STAR, f); /* first guard is not '*' */
        r = barcode_decode_frame(f);
        CHECK(!barcode_is_valid(r), "no start guard -> INVALID");

        build_frame(T_STAR, T_B, T_A, f); /* last guard is not '*' */
        r = barcode_decode_frame(f);
        CHECK(!barcode_is_valid(r), "no stop guard -> INVALID");
    }

    /* --- ambiguous character (not exactly 3 wide) -> INVALID --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS];
        barcode_t r;
        uint32_t i;
        build_frame(T_STAR, T_A, T_STAR, f);
        for (i = 10; i < 19; i++) f[i] = NARROW; /* wipe middle char: 0 wide */
        r = barcode_decode_frame(f);
        CHECK(!barcode_is_valid(r), "all-narrow middle char -> INVALID");
    }

    /* --- classify_char: exactly 3 wide required --- */
    {
        uint32_t elems[9];
        uint16_t pat = 0;
        uint8_t ok;
        emit_char(T_A, elems);
        ok = barcode_classify_char(elems, &pat);
        CHECK(ok, "classify A succeeds");
        CHECK_EQ_INT(pat, T_A, "classify A pattern matches");

        { int i; for (i = 0; i < 9; i++) elems[i] = NARROW; }
        ok = barcode_classify_char(elems, &pat);
        CHECK(!ok, "classify all-equal fails");
    }

    /* --- edges_to_widths: N edges -> N-1 widths --- */
    {
        barcode_edge_t edges[4] = { {0}, {10}, {13}, {22} };
        uint32_t w[BARCODE_FRAME_ELEMENTS];
        uint32_t n = barcode_edges_to_widths(edges, 4, w);
        CHECK_EQ_INT(n, 3, "4 edges -> 3 widths");
        CHECK_EQ_INT(w[0], 10, "width0 = 10");
        CHECK_EQ_INT(w[1], 3,  "width1 = 3");
        CHECK_EQ_INT(w[2], 9,  "width2 = 9");
    }

    /* --- edge queue: FIFO, overflow drops + counts (G3) --- */
    {
        barcode_edge_queue_t q;
        barcode_edge_t e;
        uint32_t i;
        uint8_t ok;

        barcode_eq_init(&q);
        CHECK_EQ_INT(barcode_eq_count(&q), 0, "queue starts empty");

        /* capacity is LEN-1 usable slots in a ring buffer */
        for (i = 0; i < BARCODE_EDGE_QUEUE_LEN - 1u; i++) {
            ok = barcode_eq_push(&q, i);
            CHECK(ok, "push within capacity ok");
        }
        /* next push overflows */
        ok = barcode_eq_push(&q, 999u);
        CHECK(!ok, "push on full returns 0");
        CHECK_EQ_INT(q.dropped, 1, "overflow counted");

        /* FIFO order preserved */
        ok = barcode_eq_pop(&q, &e);
        CHECK(ok, "pop ok");
        CHECK_EQ_INT(e.t_us, 0, "first out = first in");
    }

    /* ============================================================= */
    /* Pure frame-assembler tests (the task-loop logic, host-tested) */
    /* ============================================================= */

    /* 1) one *A* barcode => exactly 1 VALID, 0 INVALID */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        uint32_t f[BARCODE_FRAME_ELEMENTS], ed[BARCODE_FRAME_EDGES];
        int v = 0, inv = 0; uint8_t cmd = 0xFFu;
        build_frame(T_STAR, T_A, T_STAR, f);
        frame_to_edges(f, 100000u, 5000u, ed);
        run_edges(&a, ed, BARCODE_FRAME_EDGES, &v, &inv, &cmd);
        CHECK_EQ_INT(v, 1, "one A barcode -> exactly 1 VALID");
        CHECK_EQ_INT(inv, 0, "one A barcode -> 0 INVALID");
        CHECK_EQ_INT(cmd, L('A'), "assembler: A -> letter 'A'");
    }

    /* 2) a reversed read => 1 VALID */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        uint32_t f[BARCODE_FRAME_ELEMENTS], rev[BARCODE_FRAME_ELEMENTS];
        uint32_t ed[BARCODE_FRAME_EDGES];
        int v = 0, inv = 0; uint8_t cmd = 0xFFu;
        build_frame(T_STAR, T_B, T_STAR, f);
        reverse_frame(f, rev);
        frame_to_edges(rev, 100000u, 5000u, ed);
        run_edges(&a, ed, BARCODE_FRAME_EDGES, &v, &inv, &cmd);
        CHECK_EQ_INT(v, 1, "reversed read -> 1 VALID");
        CHECK_EQ_INT(inv, 0, "reversed read -> 0 INVALID");
        CHECK_EQ_INT(cmd, L('B'), "assembler: reversed B -> letter 'B'");
    }

    /* 3) two barcodes back to back => 2 VALID */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        uint32_t f[BARCODE_FRAME_ELEMENTS], ed[BARCODE_FRAME_EDGES];
        int v = 0, inv = 0; uint8_t cmd = 0xFFu;
        build_frame(T_STAR, T_C, T_STAR, f);
        frame_to_edges(f, 100000u, 5000u, ed);
        run_edges(&a, ed, BARCODE_FRAME_EDGES, &v, &inv, &cmd);
        /* second barcode starts after the first one ends (141 units x 5 ms
           = 705 ms long, so it ends at 805 ms) */
        frame_to_edges(f, 1000000u, 5000u, ed);
        run_edges(&a, ed, BARCODE_FRAME_EDGES, &v, &inv, &cmd);
        CHECK_EQ_INT(v, 2, "two barcodes -> 2 VALID");
        CHECK_EQ_INT(inv, 0, "two barcodes -> 0 INVALID");
    }

    /* 4) partial frame then silence => exactly 1 INVALID */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        uint32_t f[BARCODE_FRAME_ELEMENTS], ed[BARCODE_FRAME_EDGES];
        int v = 0, inv = 0; uint8_t cmd = 0xFFu;
        barcode_t out;
        uint8_t fired;
        build_frame(T_STAR, T_A, T_STAR, f);
        frame_to_edges(f, 100000u, 5000u, ed);
        /* feed only the first 12 edges, then go quiet */
        run_edges(&a, ed, 12u, &v, &inv, &cmd);
        CHECK_EQ_INT(v, 0, "partial frame -> no VALID yet");
        CHECK_EQ_INT(inv, 0, "partial frame -> no INVALID yet");

        /* first 12 edges hold a wide element of 9 x 5 ms = 45 ms, so the
           adaptive timeout is 6 x 45 = 270 ms (above the 150 ms floor) */
        CHECK_EQ_INT(barcode_asm_abort_timeout_us(&a), 270000, "abort timeout = 6 x longest (45 ms)");
        fired = barcode_asm_time_passed(&a, ed[11] + 200000u, &out); /* 200 ms */
        CHECK(!fired, "200 ms silence < 270 ms: no abandon");

        /* enough silence => exactly one INVALID */
        fired = barcode_asm_time_passed(&a, ed[11] + 300000u, &out); /* 300 ms */
        CHECK(fired, "300 ms silence > 270 ms: abandon fires");
        CHECK(!barcode_is_valid(out), "abandoned partial -> INVALID");

        /* a further time_passed must NOT fire again (reported once) */
        fired = barcode_asm_time_passed(&a, ed[11] + 600000u, &out);
        CHECK(!fired, "abandon reported only once");
    }

    /* 5) a stray noise edge => no VALID */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        barcode_t out;
        uint8_t fired = barcode_asm_edge_arrived(&a, 100000u, &out);
        CHECK(!fired, "single stray edge -> no result");
        /* no element measured yet -> the 2 s cap applies */
        fired = barcode_asm_time_passed(&a, 100000u + 200000u, &out);
        CHECK(!fired, "stray edge: 200 ms < 2 s cap, still waiting");
        fired = barcode_asm_time_passed(&a, 100000u + 2100000u, &out);
        CHECK(fired, "stray edge abandoned after the 2 s cap");
        CHECK(!barcode_is_valid(out), "stray edge -> never VALID");
    }

    /* 6) queue overflow forces INVALID and sets the fault flag.
          The assembler doesn't own the queue, so this mirrors the task's
          publish_result() overflow rule directly. */
    /* 6) queue overflow is a COUNTER that forces INVALID (G3), detected
          without the consumer ever writing q->dropped (CR1: no race). */
    {
        barcode_edge_queue_t q;
        barcode_t result;
        uint8_t seen = 0u;
        uint32_t i;
        uint8_t delta;

        barcode_eq_init(&q);
        for (i = 0; i < BARCODE_EDGE_QUEUE_LEN + 5u; i++) {
            (void)barcode_eq_push(&q, i); /* force overflow */
        }
        CHECK(q.dropped > 0, "overflow recorded dropped edges (ISR-side)");

        /* consumer detects drops via its own last-seen copy only */
        delta = barcode_eq_overflow_delta(&q, &seen);
        CHECK(delta > 0, "overflow_delta reports new drops");
        CHECK_EQ_INT(seen, q.dropped, "last-seen updated to current count");

        /* mirror the task's rule: drops since last frame -> INVALID */
        result = barcode_pack_letter(0u); /* pretend a decode happened */
        if (delta != 0u) {
            result = barcode_invalid();
        }
        CHECK(!barcode_is_valid(result), "overflow forces INVALID (counter, not fault bit)");

        /* a second check with no new drops reports zero and leaves a VALID */
        delta = barcode_eq_overflow_delta(&q, &seen);
        CHECK_EQ_INT(delta, 0, "no new drops -> delta 0");

        /* the consumer never wrote q->dropped (single-writer invariant) */
        CHECK(q.dropped > 0, "q->dropped untouched by consumer");
    }

    /* 7) slow run (0.1 m/s, wide bar ~90 ms) does not trip quiet timeout.
          us_per_unit chosen so a NARROW(3u)=30ms, WIDE(9u)=90ms. The
          largest inter-edge gap is one WIDE element = 90 ms < 150 ms. */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        uint32_t f[BARCODE_FRAME_ELEMENTS], ed[BARCODE_FRAME_EDGES];
        int v = 0, inv = 0; uint8_t cmd = 0xFFu;
        uint32_t i;
        barcode_t out;
        build_frame(T_STAR, T_D, T_STAR, f);
        frame_to_edges(f, 100000u, 10000u, ed); /* 1 unit = 10 ms => WIDE=90ms */

        /* interleave a time_passed between each edge; none should abandon */
        for (i = 0; i < BARCODE_FRAME_EDGES; i++) {
            uint8_t fired;
            if (barcode_asm_edge_arrived(&a, ed[i], &out)) {
                if (barcode_is_valid(out)) {
                    v++;
                    cmd = barcode_get_letter(out);
                } else {
                    inv++;
                }
            }
            if (i + 1 < BARCODE_FRAME_EDGES) {
                /* time advances to just before the next edge */
                fired = barcode_asm_time_passed(&a, ed[i + 1] - 1u, &out);
                CHECK(!fired, "slow run: no premature abandon between bars");
            }
        }
        /* last edge accepted after its 2 ms chatter check */
        if (barcode_asm_time_passed(&a, ed[BARCODE_FRAME_EDGES - 1] + BARCODE_CHATTER_US, &out)) {
            if (barcode_is_valid(out)) {
                v++;
                cmd = barcode_get_letter(out);
            } else {
                inv++;
            }
        }
        CHECK_EQ_INT(v, 1, "slow run -> 1 VALID (no timeout trip)");
        CHECK_EQ_INT(inv, 0, "slow run -> 0 INVALID");
        CHECK_EQ_INT(cmd, L('D'), "slow D -> letter 'D'");
    }

    /* ============================================================= */
    /* Size / speed / noise robustness (physical-scan simulation)    */
    /* ============================================================= */
    printf("\n== size, speed and noise tests ==\n");

    /* A) every letter, both directions, 5 print sizes x 3 speeds.
          0.5x = 70 mm long (1.5 mm narrow) ... 2.1x = 297 mm (A4 long side).
          30 mm/s = slow hand scan, 300 mm/s = fastest supported. */
    {
        static const double scales[] = { 0.5, 0.75, 1.0, 1.5, 2.1 };
        static const double speeds[] = { 30.0, 100.0, 300.0 };
        int total = 0, clean = 0, wrong = 0;
        uint32_t si, vi, dir, k;
        for (si = 0; si < 5u; si++) {
            for (vi = 0; vi < 3u; vi++) {
                for (dir = 0; dir < 2u; dir++) {
                    for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                        stream_t st = { {0}, 0u, 0u };
                        scan_t sc = scan(scales[si], speeds[vi]);
                        tally_t r;
                        (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &sc, 1000000u);
                        r = feed(&st, (uint8_t)k, 0u);
                        total++;
                        wrong += r.wrong;
                        if (r.right == 1 && r.wrong == 0 && r.invalid == 0) clean++;
                    }
                }
            }
        }
        printf("  info: %d of %d scans decoded (5 sizes x 3 speeds x 2 dirs x 26)\n", clean, total);
        CHECK_EQ_INT(clean, total, "A: every size/speed/direction decodes the right letter once");
        CHECK_EQ_INT(wrong, 0, "A: no wrong letter");

        /* the same with a 1 ms poll loop (as integrated_car) at the extremes */
        {
            int ok = 1;
            for (dir = 0; dir < 2u; dir++) {
                for (k = 0; k < BARCODE_LETTER_COUNT; k += 5u) {
                    stream_t st = { {0}, 0u, 0u };
                    scan_t big = scan(2.1, 30.0), small = scan(0.5, 300.0);
                    tally_t r;
                    (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &big, 1000000u);
                    r = feed(&st, (uint8_t)k, 1000u);
                    if (r.right != 1 || r.wrong != 0 || r.invalid != 0) ok = 0;
                    st.n = 0u;
                    (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &small, 1000000u);
                    r = feed(&st, (uint8_t)k, 1000u);
                    if (r.right != 1 || r.wrong != 0 || r.invalid != 0) ok = 0;
                }
            }
            CHECK(ok, "A: A4 at 30 mm/s and half-size at 300 mm/s, 1 ms poll loop");
        }
    }

    /* B) speed changing during the scan */
    {
        static const double scales[] = { 0.5, 1.0, 2.1 };
        static const double speeds[] = { 30.0, 100.0, 300.0 };
        int total = 0, right20 = 0, wrong20 = 0, right40 = 0, wrong40 = 0, total40 = 0;
        uint32_t si, vi, dir, k, mode;
        for (si = 0; si < 3u; si++) {
            for (vi = 0; vi < 3u; vi++) {
                for (dir = 0; dir < 2u; dir++) {
                    for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                        for (mode = 0; mode < 4u; mode++) {
                            stream_t st = { {0}, 0u, 0u };
                            scan_t sc = scan(scales[si], speeds[vi]);
                            tally_t r;
                            if (mode == 0u) { sc.v_mm_s *= 0.8; sc.ramp = 1.5; }       /* 0.8v -> 1.2v */
                            if (mode == 1u) { sc.v_mm_s *= 1.2; sc.ramp = 0.8 / 1.2; } /* 1.2v -> 0.8v */
                            if (mode == 2u) { sc.wobble = 0.2; }                       /* +/-20% in every char */
                            if (mode == 3u) { sc.wobble = 0.4; }                       /* +/-40%: beyond spec */
                            (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &sc, 1000000u);
                            r = feed(&st, (uint8_t)k, 0u);
                            if (mode < 3u) {
                                total++;
                                right20 += (r.right == 1);
                                wrong20 += r.wrong;
                            } else {
                                total40++;
                                right40 += (r.right == 1);
                                wrong40 += r.wrong;
                            }
                        }
                    }
                }
            }
        }
        printf("  info: +/-20%% speed change: %d of %d decoded, %d wrong\n", right20, total, wrong20);
        printf("  info: +/-40%% speed change: %d of %d decoded, %d wrong (rest INVALID)\n",
               right40, total40, wrong40);
        CHECK_EQ_INT(wrong20, 0, "B: +/-20% speed change never gives a wrong letter");
        CHECK_EQ_INT(right20, total, "B: +/-20% speed change still decodes");
        CHECK_EQ_INT(wrong40, 0, "B: +/-40% speed change never gives a wrong letter");
    }

    /* C) stray edges before and after the barcode (rolling window) */
    {
        static const uint32_t n_strays[] = { 1u, 2u, 3u, 5u, 10u, 29u, 40u };
        static const double scales[] = { 0.5, 1.0, 2.1 };
        static const double speeds[] = { 100.0, 300.0 };
        int total = 0, right = 0, wrong = 0;
        uint32_t ni, si, vi, dir, k;
        g_rng = 777u;
        for (ni = 0; ni < 7u; ni++) {
            for (si = 0; si < 3u; si++) {
                for (vi = 0; vi < 2u; vi++) {
                    for (dir = 0; dir < 2u; dir++) {
                        for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                            stream_t st = { {0}, 0u, 0u };
                            scan_t sc = scan(scales[si], speeds[vi]);
                            uint32_t wide_us = (uint32_t)(9.0 * scales[si] / speeds[vi] * 1e6);
                            uint32_t t = 1000000u, j, end;
                            tally_t r;
                            for (j = 0; j < n_strays[ni]; j++) {          /* before */
                                push(&st, t);
                                t += rng_range(2500u, 3u * wide_us);
                            }
                            end = add_barcode(&st, (uint8_t)k, (uint8_t)dir, &sc, t);
                            t = end + rng_range(2500u, 3u * wide_us);
                            for (j = 0; j < n_strays[ni]; j++) {          /* after */
                                push(&st, t);
                                t += rng_range(2500u, 3u * wide_us);
                            }
                            r = feed(&st, (uint8_t)k, 0u);
                            total++;
                            right += (r.right == 1);
                            wrong += r.wrong;
                        }
                    }
                }
            }
        }
        printf("  info: stray edges before+after: %d of %d decoded, %d wrong\n", right, total, wrong);
        CHECK_EQ_INT(wrong, 0, "C: stray edges never give a wrong letter");
        CHECK_EQ_INT(right, total, "C: stray edges before/after do not stop the decode");
    }

    /* D) chatter: glitch pulses < 2 ms inside elements + bounce at edges */
    {
        static const double scales[] = { 0.5, 1.0, 2.1 };
        static const double speeds[] = { 100.0, 300.0 };
        int total_big = 0, right_big = 0, wrong = 0, pairs_ok = 1;
        int total_small = 0, right_small = 0;
        uint32_t si, vi, dir, k, rep;
        g_rng = 4242u;
        for (si = 0; si < 3u; si++) {
            for (vi = 0; vi < 2u; vi++) {
                for (dir = 0; dir < 2u; dir++) {
                    for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                        for (rep = 0; rep < 4u; rep++) {
                            stream_t st = { {0}, 0u, 0u };
                            scan_t sc = scan(scales[si], speeds[vi]);
                            uint32_t dur[BARCODE_FRAME_ELEMENTS];
                            uint32_t i, t = 1000000u;
                            tally_t r;
                            barcode_durations((uint8_t)k, (uint8_t)dir, &sc, dur);
                            for (i = 0; i <= BARCODE_FRAME_ELEMENTS; i++) {
                                /* the real edge, sometimes with bounce (3 edges in < 2 ms) */
                                uint32_t room = (i < BARCODE_FRAME_ELEMENTS) ? dur[i] : 100000u;
                                if (rng_range(0u, 99u) < 30u && room > 4000u) {
                                    uint32_t b1 = rng_range(100u, 900u), b2 = rng_range(100u, 900u);
                                    push(&st, t);
                                    push(&st, t + b1);
                                    push(&st, t + b1 + b2);
                                    st.pairs++;
                                    if (i < BARCODE_FRAME_ELEMENTS) {
                                        t += dur[i];
                                    }
                                    continue;   /* no glitch in an element that starts with bounce */
                                }
                                push(&st, t);
                                if (i == BARCODE_FRAME_ELEMENTS) {
                                    break;
                                }
                                if (rng_range(0u, 99u) < 50u) {
                                    st.pairs += add_glitch_inside(&st, t, dur[i]);
                                }
                                t += dur[i];
                            }
                            r = feed(&st, (uint8_t)k, 0u);
                            wrong += r.wrong;
                            if (dur[0] >= 9000u) {   /* dur[0] is a narrow bar */
                                /* elements >= ~9 ms: chatter must never matter */
                                total_big++;
                                right_big += (r.right == 1);
                                if (r.right == 1 && r.pairs_seen != st.pairs) pairs_ok = 0;
                            } else {
                                total_small++;
                                right_small += (r.right == 1);
                            }
                        }
                    }
                }
            }
        }
        printf("  info: chatter, narrow element >= 9 ms: %d of %d decoded\n", right_big, total_big);
        printf("  info: chatter, narrow element 5 ms (half size at 0.3 m/s): %d of %d decoded\n",
               right_small, total_small);
        CHECK_EQ_INT(wrong, 0, "D: chatter never gives a wrong letter");
        CHECK_EQ_INT(right_big, total_big, "D: chatter on >= 9 ms elements always decodes");
        CHECK(pairs_ok, "D: every injected glitch/bounce cancelled, no real edge lost");
    }

    /* D2) the 2 ms chatter rule never cancels a real bar at the top speed */
    {
        static const double scales[] = { 0.5, 1.0 };
        int ok = 1;
        uint32_t si, dir, k;
        for (si = 0; si < 2u; si++) {
            for (dir = 0; dir < 2u; dir++) {
                for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                    stream_t st = { {0}, 0u, 0u };
                    scan_t sc = scan(scales[si], 300.0);
                    tally_t r;
                    (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &sc, 1000000u);
                    r = feed(&st, (uint8_t)k, 0u);
                    if (r.right != 1 || r.pairs_seen != 0u || r.edges_seen != BARCODE_FRAME_EDGES) ok = 0;
                }
            }
        }
        CHECK(ok, "D2: 3 mm bar (10 ms) and 1.5 mm bar (5 ms) at 0.3 m/s: 0 edges cancelled");

        {   /* exact threshold */
            barcode_asm_t a;
            barcode_t out;
            barcode_asm_init(&a);
            (void)barcode_asm_edge_arrived(&a, 1000000u, &out);
            (void)barcode_asm_edge_arrived(&a, 1000000u + BARCODE_CHATTER_US - 1u, &out);
            CHECK_EQ_INT(a.chatter_pairs, 1, "D2: edges 1.999 ms apart -> cancelled as a pair");
            CHECK_EQ_INT(a.count + a.have_pending, 0, "D2: ...and both edges are gone");
            barcode_asm_init(&a);
            (void)barcode_asm_edge_arrived(&a, 1000000u, &out);
            (void)barcode_asm_edge_arrived(&a, 1000000u + BARCODE_CHATTER_US, &out);
            CHECK_EQ_INT(a.chatter_pairs, 0, "D2: edges exactly 2.000 ms apart -> both kept");
            CHECK_EQ_INT(a.count + a.have_pending, 2, "D2: ...1 accepted + 1 pending");
        }
    }

    /* E) noise only: never VALID */
    {
        int valid = 0;
        uint32_t seed, i;
        for (seed = 1u; seed <= 40u; seed++) {
            barcode_asm_t a;
            barcode_t out;
            uint32_t t = 1000000u;
            g_rng = seed * 99991u;
            barcode_asm_init(&a);
            for (i = 0; i < 5000u; i++) {
                uint32_t band = rng_range(0u, 3u);  /* log-spread widths 0.2 ms .. 400 ms */
                uint32_t w = (band == 0u) ? rng_range(200u, 2000u)
                           : (band == 1u) ? rng_range(2000u, 20000u)
                           : (band == 2u) ? rng_range(20000u, 100000u)
                                          : rng_range(100000u, 400000u);
                if (barcode_asm_time_passed(&a, t + w - 1u, &out) && barcode_is_valid(out)) valid++;
                t += w;
                if (barcode_asm_edge_arrived(&a, t, &out) && barcode_is_valid(out)) valid++;
            }
        }
        printf("  info: noise only: 200000 random edges, %d VALID\n", valid);
        CHECK_EQ_INT(valid, 0, "E: noise only -> never VALID");
    }

    /* F) missing bar / missing edge / partial frame / bar too short.
          F1: the damaged barcode alone -> must be INVALID.
          F2: the same followed by stray edges, so the rolling window fills
              to 30+ edges and really tries to decode the damaged data.
              Never a wrong letter. The right letter is accepted only when
              the damage is in the tail (edges 26..29): stray edges that
              land where the missing final edges were rebuild a window
              whose widths all pass every rule, so it is a genuine read. */
    {
        int wrong[2] = { 0, 0 }, right[2] = { 0, 0 }, total[2] = { 0, 0 };
        int right_not_at_end = 0;
        uint32_t dir, k, kind, idx, j, strays;
        g_rng = 31337u;
        for (strays = 0u; strays < 2u; strays++) {
            for (dir = 0; dir < 2u; dir++) {
                for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                    stream_t full = { {0}, 0u, 0u };
                    scan_t sc = scan(1.0, 100.0);
                    (void)add_barcode(&full, (uint8_t)k, (uint8_t)dir, &sc, 1000000u);
                    /* kind 0: drop edges idx, idx+1 (missing bar or space)
                       kind 1: drop edge idx (bars and spaces swap after it)
                       kind 2: keep only the first idx edges (partial frame)
                       kind 3: half size at 1 m/s: 1.5 ms bars < 2 ms, so the
                               chatter rule removes real bars (edge count changes) */
                    for (kind = 0u; kind < 4u; kind++) {
                        uint32_t first = (kind == 2u) ? 1u : 0u;
                        uint32_t last  = (kind == 0u) ? 28u : (kind == 3u) ? 0u : 29u;
                        for (idx = first; idx <= last; idx++) {
                            stream_t st = { {0}, 0u, 0u };
                            tally_t r;
                            uint32_t i;
                            if (kind == 3u) {
                                scan_t fast = scan(0.5, 1000.0);
                                (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &fast, 1000000u);
                            } else {
                                for (i = 0; i < full.n; i++) {
                                    uint8_t drop = (kind == 0u) ? (i == idx || i == idx + 1u)
                                                 : (kind == 1u) ? (i == idx)
                                                                : (i >= idx);
                                    if (!drop) push(&st, full.t[i]);
                                }
                            }
                            if (strays) {
                                uint32_t tt = st.t[st.n - 1u];
                                for (j = 0; j < 6u; j++) {
                                    tt += rng_range(5000u, 120000u);
                                    push(&st, tt);
                                }
                            }
                            r = feed(&st, (uint8_t)k, 0u);
                            total[strays]++;
                            right[strays] += r.right;
                            wrong[strays] += r.wrong;
                            if (r.right && strays && !(kind != 3u && idx >= 26u)) right_not_at_end++;
                        }
                    }
                }
            }
        }
        printf("  info: F1 %d damaged scans alone: %d VALID, %d wrong\n", total[0], right[0], wrong[0]);
        printf("  info: F2 %d damaged scans + strays: %d right (damage in edges 26..29), %d wrong\n",
               total[1], right[1], wrong[1]);
        CHECK_EQ_INT(wrong[0] + wrong[1], 0, "F: damaged frames never give a wrong letter");
        CHECK_EQ_INT(right[0], 0, "F1: damaged frames on their own are all INVALID");
        CHECK_EQ_INT(right_not_at_end, 0, "F2: right letter only when a stray replaces the last edges");
    }

    /* G) slow start: first elements narrow, then the hand slows down */
    {
        static const double slow[] = { 1.5, 1.8 };
        int ok = 1, total = 0, right25 = 0;
        uint32_t si, dir, k, vi;
        static const double speeds[] = { 30.0, 100.0 };
        for (vi = 0; vi < 2u; vi++) {
            for (si = 0; si < 2u; si++) {
                for (dir = 0; dir < 2u; dir++) {
                    for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                        stream_t st = { {0}, 0u, 0u };
                        scan_t sc = scan(2.1, speeds[vi]);
                        tally_t r;
                        sc.step_at = (dir == 0u) ? 1u : 2u;  /* right after the narrow start */
                        sc.step_factor = 1.0 / slow[si];
                        (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &sc, 1000000u);
                        r = feed(&st, (uint8_t)k, 0u);
                        if (r.right != 1 || r.wrong != 0) ok = 0;
                    }
                }
            }
            for (dir = 0; dir < 2u; dir++) {          /* beyond the rule: slows 2.5x */
                for (k = 0; k < BARCODE_LETTER_COUNT; k++) {
                    stream_t st = { {0}, 0u, 0u };
                    scan_t sc = scan(2.1, speeds[vi]);
                    tally_t r;
                    sc.step_at = (dir == 0u) ? 1u : 2u;
                    sc.step_factor = 1.0 / 2.5;
                    (void)add_barcode(&st, (uint8_t)k, (uint8_t)dir, &sc, 1000000u);
                    r = feed(&st, (uint8_t)k, 0u);
                    total++;
                    right25 += (r.right == 1);
                    if (r.wrong != 0) ok = 0;
                }
            }
        }
        printf("  info: slows 2.5x after the narrow start: %d of %d decoded (rest INVALID)\n",
               right25, total);
        CHECK(ok, "G: slows 1.5x / 1.8x after narrow start -> no abort, right letter");
    }

    /* H) worst-case time from the last edge to the result */
    {
        stream_t st = { {0}, 0u, 0u };
        scan_t sc = scan(1.0, 100.0);
        barcode_asm_t a;
        barcode_t out;
        uint32_t i, last;
        int early = 0, at2ms = 0;
        (void)add_barcode(&st, L('K'), 0u, &sc, 1000000u);
        barcode_asm_init(&a);
        for (i = 0; i < st.n; i++) {
            (void)barcode_asm_edge_arrived(&a, st.t[i], &out);
        }
        last = st.t[st.n - 1u];
        early = barcode_asm_time_passed(&a, last + BARCODE_CHATTER_US - 1u, &out);
        at2ms = barcode_asm_time_passed(&a, last + BARCODE_CHATTER_US, &out);
        CHECK(!early, "H: no result 1.999 ms after the last edge");
        CHECK(at2ms && barcode_is_valid(out) && barcode_get_char(out) == 'K',
              "H: VALID exactly 2 ms after the last edge");
    }

    return TEST_SUMMARY();
}

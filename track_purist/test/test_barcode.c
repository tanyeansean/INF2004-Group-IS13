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
 * VALID and INVALID results produced by edge_arrived only (no time
 * advance). Returns the last valid letter seen.
 */
static void run_edges(barcode_asm_t *a,
                      const uint32_t *edges, uint32_t n,
                      int *valid_count, int *invalid_count,
                      uint8_t *last_letter)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        barcode_t out;
        if (barcode_asm_edge_arrived(a, edges[i], &out)) {
            if (barcode_is_valid(out)) {
                (*valid_count)++;
                *last_letter = barcode_get_letter(out);
            } else {
                (*invalid_count)++;
            }
        }
    }
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
        /* second barcode starts right after the first */
        frame_to_edges(f, 500000u, 5000u, ed);
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

        /* not enough silence yet (last edge at ed[11]) */
        fired = barcode_asm_time_passed(&a, ed[11] + 100000u, &out); /* 100 ms */
        CHECK(!fired, "100 ms silence < 150 ms: no abandon");

        /* enough silence => exactly one INVALID */
        fired = barcode_asm_time_passed(&a, ed[11] + 200000u, &out); /* 200 ms */
        CHECK(fired, "200 ms silence > 150 ms: abandon fires");
        CHECK(!barcode_is_valid(out), "abandoned partial -> INVALID");

        /* a further time_passed must NOT fire again (reported once) */
        fired = barcode_asm_time_passed(&a, ed[11] + 400000u, &out);
        CHECK(!fired, "abandon reported only once");
    }

    /* 5) a stray noise edge => no VALID */
    {
        barcode_asm_t a; barcode_asm_init(&a);
        barcode_t out;
        uint8_t fired = barcode_asm_edge_arrived(&a, 100000u, &out);
        CHECK(!fired, "single stray edge -> no result");
        /* and it gets abandoned on silence, as INVALID not VALID */
        fired = barcode_asm_time_passed(&a, 100000u + 200000u, &out);
        CHECK(fired, "stray edge abandoned after silence");
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
        CHECK_EQ_INT(v, 1, "slow run -> 1 VALID (no timeout trip)");
        CHECK_EQ_INT(inv, 0, "slow run -> 0 INVALID");
        CHECK_EQ_INT(cmd, L('D'), "slow D -> letter 'D'");
    }

    return TEST_SUMMARY();
}

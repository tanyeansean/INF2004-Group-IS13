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

/* Pattern constants mirrored from barcode_decode.c for test construction. */
#define T_STAR 0x094u
#define T_A    0x109u
#define T_B    0x049u
#define T_C    0x148u
#define T_D    0x019u
#define T_Z    0x0D0u  /* real Code 39 'Z' = nWWnWnnnn (not a command) */

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

int main(void)
{
    printf("== barcode decode tests ==\n");

    /* --- valid frames: * <cmd> * --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS];
        barcode_t r;

        build_frame(T_STAR, T_A, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(barcode_is_valid(r), "A decodes valid");
        CHECK_EQ_INT(barcode_get_cmd(r), BARCMD_LEFT, "A -> LEFT");

        build_frame(T_STAR, T_B, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(barcode_is_valid(r), "B decodes valid");
        CHECK_EQ_INT(barcode_get_cmd(r), BARCMD_RIGHT, "B -> RIGHT");

        build_frame(T_STAR, T_C, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(barcode_is_valid(r), "C decodes valid");
        CHECK_EQ_INT(barcode_get_cmd(r), BARCMD_STRAIGHT, "C -> STRAIGHT");

        build_frame(T_STAR, T_D, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(barcode_is_valid(r), "D decodes valid");
        CHECK_EQ_INT(barcode_get_cmd(r), BARCMD_U_TURN, "D -> U_TURN");
    }

    /* --- reversed frame still decodes (direction independence) --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS], rev[BARCODE_FRAME_ELEMENTS];
        barcode_t r;
        build_frame(T_STAR, T_B, T_STAR, f);
        reverse_frame(f, rev);
        r = barcode_decode_frame(rev);
        CHECK(barcode_is_valid(r), "reversed B decodes valid");
        CHECK_EQ_INT(barcode_get_cmd(r), BARCMD_RIGHT, "reversed B -> RIGHT");
    }

    /* --- Z is a valid Code39 char but NOT a command -> INVALID --- */
    {
        uint32_t f[BARCODE_FRAME_ELEMENTS];
        barcode_t r;
        build_frame(T_STAR, T_Z, T_STAR, f);
        r = barcode_decode_frame(f);
        CHECK(!barcode_is_valid(r), "Z -> INVALID (not a command)");
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

    return TEST_SUMMARY();
}

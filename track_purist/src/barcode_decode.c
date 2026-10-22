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
/* '*' start/stop guard. */
#define PAT_STAR  0x094u /* 0 1001 0100 */

/* Four command characters (placeholder mapping A/B/C/D -> commands). */
#define PAT_A     0x109u /* 1 0000 1001 */
#define PAT_B     0x049u /* 0 0100 1001 */
#define PAT_C     0x148u /* 1 0100 1000 */
#define PAT_D     0x019u /* 0 0001 1001 */
/* 'Z' is intentionally NOT a command -> must decode as INVALID. */
#define PAT_Z     0x0A2u /* 0 0101 0010 */

char barcode_pattern_to_char(uint16_t pattern9)
{
    pattern9 &= 0x1FFu;
    switch (pattern9) {
        case PAT_STAR: return '*';
        case PAT_A:    return 'A';
        case PAT_B:    return 'B';
        case PAT_C:    return 'C';
        case PAT_D:    return 'D';
        default:       return 0;   /* unknown (includes PAT_Z) */
    }
}

uint8_t barcode_char_to_cmd(char c, barcode_cmd_t *cmd)
{
    switch (c) {
        case 'A': *cmd = BARCMD_LEFT;     return 1u;
        case 'B': *cmd = BARCMD_RIGHT;    return 1u;
        case 'C': *cmd = BARCMD_STRAIGHT; return 1u;
        case 'D': *cmd = BARCMD_U_TURN;   return 1u;
        default:  return 0u;
    }
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
        if (elems[i] < mn) mn = elems[i];
        if (elems[i] > mx) mx = elems[i];
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
 *   Valid iff char0 == '*' and char2 == '*' and char1 is a command.
 */
static barcode_t decode_oriented(const uint32_t w[BARCODE_FRAME_ELEMENTS])
{
    uint16_t p0, p1, p2;
    char c0, c1, c2;
    barcode_cmd_t cmd;

    if (!barcode_classify_char(&w[0],  &p0)) return barcode_invalid();
    if (!barcode_classify_char(&w[10], &p1)) return barcode_invalid();
    if (!barcode_classify_char(&w[20], &p2)) return barcode_invalid();

    c0 = barcode_pattern_to_char(p0);
    c1 = barcode_pattern_to_char(p1);
    c2 = barcode_pattern_to_char(p2);

    if (c0 != '*' || c2 != '*') return barcode_invalid();
    if (!barcode_char_to_cmd(c1, &cmd)) return barcode_invalid();

    return barcode_pack(1u, cmd);
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
    if (n_edges < 2u) return 0u;
    n = n_edges - 1u;
    if (n > BARCODE_FRAME_ELEMENTS) n = BARCODE_FRAME_ELEMENTS;
    for (i = 0u; i < n; i++) {
        widths_out[i] = edges[i + 1u].t_us - edges[i].t_us;
    }
    return n;
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
    uint32_t next = (q->head + 1u) % BARCODE_EDGE_QUEUE_LEN;
    if (next == q->tail) {
        q->dropped++;        /* overflow: drop the edge, count it (G3) */
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
    q->tail = (q->tail + 1u) % BARCODE_EDGE_QUEUE_LEN;
    return 1u;
}

uint32_t barcode_eq_count(const barcode_edge_queue_t *q)
{
    return (q->head + BARCODE_EDGE_QUEUE_LEN - q->tail) % BARCODE_EDGE_QUEUE_LEN;
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
extern rtos_flag_t  g_fault_flag;
extern uint32_t     g_barcode_hb_bit;
extern uint32_t     g_barcode_fault_ovf_bit; /* maps to FAULT_BARCODE_OVF slot */

/* Board hook: current microsecond timestamp (e.g. time_us_32()). */
extern uint32_t board_now_us(void);

/* Right-IR GPIO edge callback: timestamp only, no interpretation. */
void barcode_edge_isr(void)
{
    (void)barcode_eq_push(&g_barcode_eq, board_now_us());
}

/*
 * Decode task: gather edges into a frame. A frame is considered complete
 * when a quiet gap (no new edge within BARCODE_FRAME_TIMEOUT_MS) follows
 * at least 2 edges. The 20 ms wait also lets the task set its heartbeat
 * bit regularly even when no barcode is present.
 */
void barcode_decode_task(void)
{
    barcode_edge_t edges[BARCODE_EDGE_QUEUE_LEN];
    uint32_t count = 0u;

    for (;;) {
        barcode_edge_t e;
        uint8_t got = barcode_eq_pop(&g_barcode_eq, &e);

        if (got) {
            if (count < BARCODE_EDGE_QUEUE_LEN) {
                edges[count++] = e;
            }
        } else {
            /* No edge right now: pace at the frame timeout + heartbeat. */
            (void)rtos_flag_set(g_hb_flag, g_barcode_hb_bit);

            if (count >= 2u) {
                /* Quiet period after some edges => frame boundary. */
                uint32_t widths[BARCODE_FRAME_ELEMENTS];
                uint32_t n = barcode_edges_to_widths(edges, count, widths);
                barcode_t result;

                if (n == BARCODE_FRAME_ELEMENTS) {
                    result = barcode_decode_frame(widths);
                } else {
                    result = barcode_invalid();
                }

                /* If the edge queue overflowed during capture, flag fault
                   and force INVALID so a partial frame is never trusted. */
                if (g_barcode_eq.dropped != 0u) {
                    (void)rtos_flag_set(g_fault_flag, g_barcode_fault_ovf_bit);
                    result = barcode_invalid();
                    g_barcode_eq.dropped = 0u;
                }

                (void)rtos_queue_send(g_barcode_q, &result, sizeof(result));
                count = 0u;
            }

            rtos_delay_ms(BARCODE_FRAME_TIMEOUT_MS);
        }
    }
}
#endif /* BARCODE_HOST_TEST */

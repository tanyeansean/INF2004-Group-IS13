/*
 * track_purist_selftest.c  -  Buddy 3 bare-metal self-test (NO uT-Kernel).
 *
 * Purpose: flash this to the Pico and watch, over USB serial, what the
 * IR sensors see and what your tested logic decides. This reuses the
 * real line_follow / barcode_decode / ir_calibrate code unchanged; it
 * only replaces the RTOS plumbing with a plain while(1) + printf.
 *
 * Wiring (Robo Pico Grove ports, per official datasheet Rev 1.0):
 *   Left-center  IR A0 -> Grove 6 YELLOW wire -> GP27 / ADC1   (prints as L=)
 *   Right-center IR A0 -> Grove 5 YELLOW wire -> GP26 / ADC0   (prints as C=)
 *   Both sensors: red (3V3) -> VCC, black -> GND. White wires and D0 unused.
 *   Barcode IR D0      -> Grove 7 WHITE wire  -> GP7 (digital, LEFT side)
 *                         NOT plugged in yet: disabled via ENABLE_BARCODE.
 *
 * NOTE: Grove 6's white wire is ALSO GP26 (same pin as Grove 5 yellow),
 * so it must stay unplugged or it will short against the right sensor.
 *
 * In the line_follow API the two sensors are called "left" and "center";
 * here "left" = left-center sensor, "center" = right-center sensor.
 *
 * How to read the output:
 *   - Calibration: follow the prompts. Each phase gives a 3 s countdown
 *     to position the sensors, then samples for 1.5 s.
 *   - Then it prints raw readings, line error, junction and lost ~5x/s.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "line_follow.h"
#include "ir_calibrate.h"
#include "barcode_decode.h"
#include "nav_cmd.h"

/* ---- Pin configuration (Robo Pico Grove ports) ---- */
#define PIN_LEFT_ADC    27   /* Grove 6 yellow, left-center sensor,  ADC1 */
#define PIN_CENTER_ADC  26   /* Grove 5 yellow, right-center sensor, ADC0 */
#define ADC_CH_LEFT      1   /* GP27 -> ADC1 */
#define ADC_CH_CENTER    0   /* GP26 -> ADC0 */
#define PIN_BARCODE      7   /* Grove 7 white, barcode D0 (LEFT side) */

/* Set to 1 once the barcode sensor is plugged into Grove 7. While 0 the
   GP7 interrupt is not enabled, so a floating pin cannot spam edges. */
#ifndef ENABLE_BARCODE
#define ENABLE_BARCODE   1
#endif

/* Minimum |line mean - background mean| (12-bit ADC counts) for a sensor
   to count as "actually seeing" the line. Below this, wiring/height/pot
   is probably wrong. */
#define CAL_MIN_CONTRAST 200u

#define CAL_COUNTDOWN_S   3
#define CAL_SAMPLES     150   /* x 10 ms = 1.5 s per phase */

/* ---- Board hooks (plain functions, no RTOS) ---- */
static uint16_t read_left_adc(void)   { adc_select_input(ADC_CH_LEFT);   return (uint16_t)adc_read(); }
static uint16_t read_center_adc(void) { adc_select_input(ADC_CH_CENTER); return (uint16_t)adc_read(); }

#if ENABLE_BARCODE
/* ---- Barcode edge queue + ISR ---- */
static barcode_edge_queue_t g_eq;
static volatile uint32_t g_edge_total; /* self-test diagnostic only */

static void barcode_ir_isr(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    g_edge_total++;
    (void)barcode_eq_push(&g_eq, time_us_32()); /* timestamp only */
}
#endif

/* 1 = after every barcode result print the window edge count, edges in the
   burst, glitch pulses cancelled, longest element and all widths in ms.
   The decoder adapts to print size and hand/car speed by itself (no
   hand-test timeout needed any more). */
#ifndef BARCODE_DEBUG
#define BARCODE_DEBUG 1
#endif

/* ---- Calibration helpers ---- */
static void countdown(const char *msg)
{
    printf("%s\n", msg);
    for (int s = CAL_COUNTDOWN_S; s > 0; s--) {
        printf("  starting in %d...\n", s);
        sleep_ms(1000);
    }
    printf("  SAMPLING - hold still (1.5 s)\n");
}

static uint32_t line_mean(const ir_cal_accum_t *a) { return a->line_count ? a->line_sum / a->line_count : 0u; }
static uint32_t bg_mean(const ir_cal_accum_t *a)   { return a->bg_count   ? a->bg_sum   / a->bg_count   : 0u; }

static uint32_t abs_diff(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }

int main(void)
{
    stdio_init_all();

    /* Wait (up to 15 s) for a serial terminal so the calibration prompts
       are not missed, then a short settle. */
    absolute_time_t deadline = make_timeout_time_ms(15000);
    while (!stdio_usb_connected() && !time_reached(deadline)) {
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n=== Buddy 3 Track Purist self-test ===\n");
    printf("Line logic: STRADDLE v3 (both white = centred, last_side, time-based lost OFF)\n");
    printf("Barcode: Code 39 A-Z, both scan directions; letter->move map NOT hard-coded\n");
    printf("L = left-center  (Grove 6, GP27/ADC1)\n");
    printf("C = right-center (Grove 5, GP26/ADC0)\n");
    printf("Barcode (Grove 7, GP7): %s\n", ENABLE_BARCODE ? "ENABLED" : "disabled");

    /* ADC setup */
    adc_init();
    adc_gpio_init(PIN_LEFT_ADC);
    adc_gpio_init(PIN_CENTER_ADC);

#if ENABLE_BARCODE
    /* Barcode IR edge interrupt (pull-up so an idle line is defined) */
    barcode_eq_init(&g_eq);
    gpio_init(PIN_BARCODE);
    gpio_set_dir(PIN_BARCODE, GPIO_IN);
    gpio_pull_up(PIN_BARCODE);
    gpio_set_irq_enabled_with_callback(PIN_BARCODE,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &barcode_ir_isr);
#endif

    /* ---- Calibration (manual, prompted) ---- */
    ir_cal_accum_t cal_l, cal_c;
    line_follow_cfg_t cfg;
    ir_cal_reset(&cal_l);
    ir_cal_reset(&cal_c);

    countdown("CALIBRATE 1/2: put BOTH sensors over the BLACK LINE");
    for (int i = 0; i < CAL_SAMPLES; i++) {
        ir_cal_add_line(&cal_l, read_left_adc());
        ir_cal_add_line(&cal_c, read_center_adc());
        sleep_ms(10);
    }

    countdown("CALIBRATE 2/2: put BOTH sensors over the BACKGROUND (white)");
    for (int i = 0; i < CAL_SAMPLES; i++) {
        ir_cal_add_background(&cal_l, read_left_adc());
        ir_cal_add_background(&cal_c, read_center_adc());
        sleep_ms(10);
    }

    cfg.junction_hold_ms = 30;
    cfg.error_gain = 1;
    cfg.lost_timeout_ms = 0; /* time-based lost OFF: both white = centred */
    if (!ir_cal_apply(&cal_l, &cal_c, &cfg)) {
        printf("CALIBRATION FAILED (no samples). Halting.\n");
        while (1) { tight_loop_contents(); }
    }

    /* Sanity checks ir_cal_apply does not do: real contrast per sensor,
       and both sensors agreeing on polarity (cfg uses the left one's). */
    uint32_t ll = line_mean(&cal_l), lb = bg_mean(&cal_l);
    uint32_t cl = line_mean(&cal_c), cb = bg_mean(&cal_c);
    printf("L: black mean=%4lu  white mean=%4lu  thr=%u\n",
           (unsigned long)ll, (unsigned long)lb, cfg.left_threshold);
    printf("C: black mean=%4lu  white mean=%4lu  thr=%u\n",
           (unsigned long)cl, (unsigned long)cb, cfg.center_threshold);
    printf("dark_is_high=%u\n", cfg.dark_is_high);

    uint8_t cal_ok = 1;
    if (abs_diff(ll, lb) < CAL_MIN_CONTRAST) {
        printf("WARNING: L has low contrast - check Grove 6 yellow->A0, sensor height, 3V3/GND.\n");
        cal_ok = 0;
    }
    if (abs_diff(cl, cb) < CAL_MIN_CONTRAST) {
        printf("WARNING: C has low contrast - check Grove 5 yellow->A0, sensor height, 3V3/GND.\n");
        cal_ok = 0;
    }
    if ((ll > lb) != (cl > cb)) {
        printf("WARNING: L and C disagree on polarity (one reads higher on black, one lower).\n");
        cal_ok = 0;
    }
    printf("Calibration %s. Running...\n", cal_ok ? "OK" : "SUSPECT (still running so you can debug)");

    /* ---- Run loop ---- */
    line_follow_state_t ls;
    line_follow_reset(&ls);

    /* Navigation: mapping arrives over MQTT in the real build; here you
       type it on the serial monitor and it goes through the SAME
       nav_map_apply() path. */
    nav_map_t   nav_map;
    nav_state_t nav;
    nav_map_init(&nav_map);
    nav_reset(&nav);
    char    map_buf[8];
    uint8_t map_len = 0;
    uint32_t last_rx_ms = 0;
    printf("MAP: type 4 letters (Enter optional), order LEFT RIGHT FORWARD UTURN (e.g. FZWX)\n");
    printf("     test without a barcode: type '#' + letter (e.g. #Z) to fake a barcode\n");

#if ENABLE_BARCODE
    barcode_asm_t basm;
    barcode_asm_init(&basm);
    printf("Barcode: any print size up to A4, any steady speed. Watch bc=0/1 flip on each bar.\n");
    uint8_t seen_dropped = 0;
    (void)barcode_eq_overflow_delta(&g_eq, &seen_dropped);
#endif

    uint32_t last_print = 0;

    while (1) {
        /* ----- serial input: mapping / fake barcode -----
           A command ends on Enter OR after 300 ms with no new character,
           so it works even if the serial monitor sends no line ending. */
        int ch = getchar_timeout_us(0);
        uint8_t end_cmd = 0;
        uint32_t now_rx = to_ms_since_boot(get_absolute_time());
        while (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                end_cmd = (map_len != 0);
                break;
            }
            if (map_len < sizeof(map_buf)) {
                map_buf[map_len++] = (char)ch;
            }
            last_rx_ms = now_rx;
            ch = getchar_timeout_us(0);
        }
        if (!end_cmd && map_len != 0 && (now_rx - last_rx_ms) >= 300u) {
            end_cmd = 1;
        }
        if (end_cmd) {
            printf("\n========== RX: \"%.*s\" ==========\n", map_len, map_buf);
            {
                if (map_len == 2 && map_buf[0] == '#') {
                    char c = (char)(map_buf[1] & ~0x20); /* upper-case */
                    if (c >= 'A' && c <= 'Z') {
                        printf(">>> BARCODE (fake): '%c' stored, waiting for junction\n", c);
                        nav_on_barcode(&nav, barcode_pack_letter((uint8_t)(c - 'A')));
                    } else {
                        printf("? after # type one letter A-Z\n");
                    }
                } else if (map_len == BARCMD_COUNT) {
                    uint8_t letters[BARCMD_COUNT];
                    uint8_t payload[NAV_MAP_PAYLOAD_LEN];
                    uint8_t st = NAV_MAP_OK;
                    for (uint8_t i = 0; i < BARCMD_COUNT; i++) {
                        char c = (char)(map_buf[i] & ~0x20);
                        if (c < 'A' || c > 'Z') { st = NAV_MAP_ERR_RANGE; break; }
                        letters[i] = (uint8_t)(c - 'A');
                    }
                    if (st == NAV_MAP_OK) { st = nav_map_encode(letters, payload); }
                    if (st == NAV_MAP_OK) { st = nav_map_apply(&nav_map, payload, NAV_MAP_PAYLOAD_LEN); }
                    if (st == NAV_MAP_OK) {
                        printf("MAP OK: LEFT=%c RIGHT=%c FORWARD=%c UTURN=%c (payload %02X %02X %02X)\n",
                               'A' + letters[0], 'A' + letters[1], 'A' + letters[2], 'A' + letters[3],
                               payload[0], payload[1], payload[2]);
                    } else {
                        printf("MAP REJECTED (err %u): need 4 different letters A-Z\n", st);
                    }
                } else {
                    printf("? type 4 letters (map, e.g. ABCD) or #<letter> (fake barcode)\n");
                }
                printf("==========================================\n\n");
                map_len = 0;
            }
        }

        /* ----- line following ----- */
        line_sample_t s;
        s.left_adc   = read_left_adc();
        s.center_adc = read_center_adc();
        s.now_ms     = to_ms_since_boot(get_absolute_time());
        line_err_t le = line_follow_step(&cfg, &ls, &s);

        /* ----- junction: execute the stored barcode once ----- */
        {
            static const char *move_names[BARCMD_COUNT] = { "LEFT", "RIGHT", "FORWARD", "U_TURN" };
            uint8_t mv = 0, let = 0;
            uint8_t ev = nav_on_line(&nav, le, nav_map_snapshot(&nav_map), &mv, &let);
            if (ev == NAV_EVT_MOVE) {
                printf(">>> JUNCTION: barcode '%c' -> %s\n", 'A' + let, move_names[mv & 3]);
            } else if (ev == NAV_EVT_UNMAPPED) {
                printf(">>> JUNCTION: barcode '%c' has NO mapping (send/type the map)\n", 'A' + let);
            }
        }

        /* print line state ~5 Hz so it is readable */
        if (s.now_ms - last_print >= 200) {
            uint8_t pend = 0;
            uint8_t has = nav_has_pending(&nav, &pend);
            last_print = s.now_ms;
            printf("L=%4u(%c) C=%4u(%c) | err=%4d junction=%u lost=%u last=%c raw=0x%04X | pending=%c",
                   s.left_adc,   line_follow_on_black(&cfg, s.left_adc, 0u)   ? 'B' : 'W',
                   s.center_adc, line_follow_on_black(&cfg, s.center_adc, 1u) ? 'B' : 'W',
                   line_err_get_error(le),
                   line_err_get_junction(le),
                   line_err_get_lost(le),
                   line_err_get_last_side(le) ? 'R' : 'L',
                   le.packed,
                   has ? (char)('A' + pend) : '-');
#if ENABLE_BARCODE
            printf(" | bc=%u edges=%lu", (unsigned)gpio_get(PIN_BARCODE), (unsigned long)g_edge_total);
#endif
            printf("\n");
        }

#if ENABLE_BARCODE
        /* ----- barcode ----- */
        barcode_edge_t e;
        barcode_t result;
        uint8_t got = 0;
        while (!got && barcode_eq_pop(&g_eq, &e)) {
            got = barcode_asm_edge_arrived(&basm, e.t_us, &result);
        }
        if (!got) {
            /* accepts a pending edge after 2 ms, or abandons a quiet frame */
            got = barcode_asm_time_passed(&basm, time_us_32(), &result);
        }
        if (got) {
            barcode_frame_info_t info;
            barcode_asm_last_frame(&basm, &info);
            if (barcode_eq_overflow_delta(&g_eq, &seen_dropped) != 0) {
                result = barcode_invalid();
            }
            if (barcode_is_valid(result)) {
                printf(">>> BARCODE: VALID '%c' stored, waiting for junction\n", barcode_get_char(result));
                nav_on_barcode(&nav, result);
            } else if (info.outcome == BARCODE_OUT_PARTIAL) {
                printf(">>> BARCODE: INVALID (partial) - got %u of 30 edges\n", (unsigned)info.n_edges);
            } else {
                printf(">>> BARCODE: INVALID - %u edges, no 30-edge window decoded\n",
                       (unsigned)info.burst_edges);
            }
#if BARCODE_DEBUG
            printf("    dbg: edges=%u burst=%u chatter=%u longest=%lu.%lu ms widths ms:",
                   (unsigned)info.n_edges, (unsigned)info.burst_edges, (unsigned)info.chatter_pairs,
                   (unsigned long)(info.longest_us / 1000u),
                   (unsigned long)((info.longest_us % 1000u) / 100u));
            for (uint32_t i = 0; i < info.n_widths; i++) {
                printf(" %lu.%lu", (unsigned long)(info.widths_us[i] / 1000u),
                       (unsigned long)((info.widths_us[i] % 1000u) / 100u));
            }
            printf("\n");
#endif
        }
#endif

        sleep_ms(5);
    }
}

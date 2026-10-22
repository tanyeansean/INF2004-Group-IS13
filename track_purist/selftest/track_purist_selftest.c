/*
 * track_purist_selftest.c  -  Buddy 3 bare-metal self-test (NO uT-Kernel).
 *
 * Purpose: flash this to the Pico and watch, over USB serial, what the
 * three IR sensors see and what your tested logic decides. This reuses
 * the real line_follow / barcode_decode / ir_calibrate code unchanged;
 * it only replaces the RTOS plumbing with a plain while(1) + printf.
 *
 * Wiring assumed (EDIT THESE to match your car):
 *   Left   IR analog -> GP26 / ADC0
 *   Center IR analog -> GP28 / ADC2
 *   Right  IR digital-> GP6   (barcode edges)   <-- CONFIRM THIS PIN
 *
 * How to read the output:
 *   - On boot it does a 3-second calibration: hold the LEFT+CENTER
 *     sensors over the BLACK line for ~1.5 s, then over the BACKGROUND
 *     for ~1.5 s (follow the serial prompts).
 *   - Then it prints line error / junction / lost every 200 ms, and
 *     prints a line whenever a barcode is decoded.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "line_follow.h"
#include "ir_calibrate.h"
#include "barcode_decode.h"

/* ---- Pin configuration (EDIT to match your wiring) ---- */
#define PIN_LEFT_ADC    26   /* ADC0 */
#define PIN_CENTER_ADC  28   /* ADC2 */
#define ADC_CH_LEFT      0
#define ADC_CH_CENTER    2
#define PIN_RIGHT_BARCODE 6  /* digital edge input - CONFIRM */

/* ---- Board hooks (plain functions, no RTOS) ---- */
static uint16_t read_left_adc(void)   { adc_select_input(ADC_CH_LEFT);   return (uint16_t)adc_read(); }
static uint16_t read_center_adc(void) { adc_select_input(ADC_CH_CENTER); return (uint16_t)adc_read(); }

/* ---- Barcode edge queue + ISR ---- */
static barcode_edge_queue_t g_eq;

static void right_ir_isr(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    (void)barcode_eq_push(&g_eq, time_us_32()); /* timestamp only */
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000); /* give the USB serial time to attach */

    printf("\n=== Buddy 3 Track Purist self-test ===\n");

    /* ADC setup */
    adc_init();
    adc_gpio_init(PIN_LEFT_ADC);
    adc_gpio_init(PIN_CENTER_ADC);

    /* Right IR edge interrupt */
    barcode_eq_init(&g_eq);
    gpio_init(PIN_RIGHT_BARCODE);
    gpio_set_dir(PIN_RIGHT_BARCODE, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_RIGHT_BARCODE,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_ir_isr);

    /* ---- Calibration (manual, prompted) ---- */
    ir_cal_accum_t cal_l, cal_c;
    line_follow_cfg_t cfg;
    ir_cal_reset(&cal_l);
    ir_cal_reset(&cal_c);

    printf("CALIBRATE: hold LEFT+CENTER over the BLACK LINE now...\n");
    for (int i = 0; i < 150; i++) {            /* ~1.5 s of line */
        ir_cal_add_line(&cal_l, read_left_adc());
        ir_cal_add_line(&cal_c, read_center_adc());
        sleep_ms(10);
    }
    printf("CALIBRATE: now hold them over the BACKGROUND...\n");
    for (int i = 0; i < 150; i++) {            /* ~1.5 s of background */
        ir_cal_add_background(&cal_l, read_left_adc());
        ir_cal_add_background(&cal_c, read_center_adc());
        sleep_ms(10);
    }

    cfg.junction_hold_ms = 30;
    cfg.error_gain = 1;
    if (!ir_cal_apply(&cal_l, &cal_c, &cfg)) {
        printf("CALIBRATION FAILED - check wiring/sensors. Halting.\n");
        while (1) { tight_loop_contents(); }
    }
    printf("Calibrated: left_thr=%u center_thr=%u dark_is_high=%u\n",
           cfg.left_threshold, cfg.center_threshold, cfg.dark_is_high);

    /* ---- Run loop ---- */
    line_follow_state_t ls;
    line_follow_reset(&ls);

    barcode_asm_t basm;
    barcode_asm_init(&basm);
    uint8_t seen_dropped = 0;
    (void)barcode_eq_overflow_delta(&g_eq, &seen_dropped);

    uint32_t last_print = 0;

    while (1) {
        /* ----- line following ----- */
        line_sample_t s;
        s.left_adc   = read_left_adc();
        s.center_adc = read_center_adc();
        s.now_ms     = to_ms_since_boot(get_absolute_time());
        line_err_t le = line_follow_step(&cfg, &ls, &s);

        /* print line state ~5 Hz so it is readable */
        if (s.now_ms - last_print >= 200) {
            last_print = s.now_ms;
            printf("L=%4u C=%4u | err=%4d junction=%u lost=%u\n",
                   s.left_adc, s.center_adc,
                   line_err_get_error(le),
                   line_err_get_junction(le),
                   line_err_get_lost(le));
        }

        /* ----- barcode ----- */
        barcode_edge_t e;
        barcode_t result;
        while (barcode_eq_pop(&g_eq, &e)) {
            if (barcode_asm_edge_arrived(&basm, e.t_us, &result)) {
                if (barcode_eq_overflow_delta(&g_eq, &seen_dropped) != 0) {
                    result = barcode_invalid();
                }
                if (barcode_is_valid(result)) {
                    const char *names[4] = { "LEFT", "RIGHT", "STRAIGHT", "U_TURN" };
                    printf(">>> BARCODE: VALID  cmd=%s\n", names[barcode_get_cmd(result) & 3]);
                } else {
                    printf(">>> BARCODE: INVALID\n");
                }
            }
        }
        /* abandon a stalled partial frame after quiet time */
        if (barcode_asm_time_passed(&basm, time_us_32(), &result)) {
            (void)barcode_eq_overflow_delta(&g_eq, &seen_dropped);
            printf(">>> BARCODE: INVALID (partial/timeout)\n");
        }

        sleep_ms(5);
    }
}

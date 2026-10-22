/**
 * @file ir_calibrate.c
 * @brief Analog IR threshold calibration (A2) — pure accumulation logic.
 */
#include "ir_calibrate.h"

void ir_cal_reset(ir_cal_accum_t *acc)
{
    acc->line_sum = 0u;
    acc->line_count = 0u;
    acc->bg_sum = 0u;
    acc->bg_count = 0u;
}

void ir_cal_add_line(ir_cal_accum_t *acc, uint16_t adc)
{
    acc->line_sum += adc;
    acc->line_count++;
}

void ir_cal_add_background(ir_cal_accum_t *acc, uint16_t adc)
{
    acc->bg_sum += adc;
    acc->bg_count++;
}

uint8_t ir_cal_compute(const ir_cal_accum_t *acc, uint16_t *threshold)
{
    uint32_t line_mean, bg_mean;

    if (acc->line_count == 0u || acc->bg_count == 0u) {
        return 0u; /* cannot calibrate without both classes */
    }
    line_mean = acc->line_sum / acc->line_count;
    bg_mean   = acc->bg_sum   / acc->bg_count;

    *threshold = (uint16_t)((line_mean + bg_mean) / 2u);
    return 1u;
}

uint8_t ir_cal_dark_is_high(const ir_cal_accum_t *acc)
{
    uint32_t line_mean, bg_mean;
    if (acc->line_count == 0u || acc->bg_count == 0u) {
        return 0u;
    }
    line_mean = acc->line_sum / acc->line_count;
    bg_mean   = acc->bg_sum   / acc->bg_count;
    return (uint8_t)(line_mean > bg_mean ? 1u : 0u);
}

uint8_t ir_cal_apply(const ir_cal_accum_t *left,
                     const ir_cal_accum_t *center,
                     line_follow_cfg_t *cfg)
{
    uint16_t lt, ct;
    if (!ir_cal_compute(left, &lt))   return 0u;
    if (!ir_cal_compute(center, &ct)) return 0u;

    cfg->left_threshold   = lt;
    cfg->center_threshold = ct;
    /* Both sensors share polarity; derive from the Left accumulator. */
    cfg->dark_is_high = ir_cal_dark_is_high(left);
    return 1u;
}

/**
 * @file ir_calibrate.h
 * @brief Analog IR threshold calibration for Left + Center (assumption A2).
 *
 * Runs in the READY state before a run. Captures ADC samples of the Left
 * and Center sensors over the black line and over the background, then
 * derives a threshold per sensor (midpoint between the two means). This
 * replaces the Week-6 guessed thresholds scaled from the sample PDF.
 *
 * Pure accumulation logic is host-testable; the ADC read itself lives in
 * the RTOS/hardware-facing wrapper.
 */
#ifndef IR_CALIBRATE_H
#define IR_CALIBRATE_H

#include <stdint.h>
#include "line_follow.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Running accumulator for one sensor's calibration. */
typedef struct {
    uint32_t line_sum;
    uint32_t line_count;
    uint32_t bg_sum;
    uint32_t bg_count;
} ir_cal_accum_t;

/** Reset an accumulator before a new calibration pass. */
void ir_cal_reset(ir_cal_accum_t *acc);

/** Add a sample taken while the sensor is over the BLACK line. */
void ir_cal_add_line(ir_cal_accum_t *acc, uint16_t adc);

/** Add a sample taken while the sensor is over the BACKGROUND. */
void ir_cal_add_background(ir_cal_accum_t *acc, uint16_t adc);

/**
 * Compute the threshold as the midpoint of the line and background
 * means. Returns 1 on success and writes *threshold; returns 0 if either
 * class has no samples (cannot calibrate).
 */
uint8_t ir_cal_compute(const ir_cal_accum_t *acc, uint16_t *threshold);

/**
 * Determine dark_is_high: 1 if the line (dark) mean reads higher than the
 * background mean, else 0. Valid only after both classes have samples.
 */
uint8_t ir_cal_dark_is_high(const ir_cal_accum_t *acc);

/**
 * Fill a line_follow_cfg_t's thresholds from two computed accumulators.
 * Returns 1 if both sensors calibrated successfully, else 0.
 */
uint8_t ir_cal_apply(const ir_cal_accum_t *left,
                     const ir_cal_accum_t *center,
                     line_follow_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* IR_CALIBRATE_H */

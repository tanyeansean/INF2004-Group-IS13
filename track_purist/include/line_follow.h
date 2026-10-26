/**
 * @file line_follow.h
 * @brief Line Follow module (Buddy 3) — Left + Center analog IR.
 *
 * Period: 5 ms (NFR1). Computes a difference-based line error from the
 * Left and Center analog readings, detects a junction (both sensors
 * black held for >= N ms) and lost-line (both on background), packs the
 * result into line_err_t and sends it to line_err_q with a zero-timeout
 * (non-blocking) send. No direct motor calls.
 *
 * The compute step is pure and host-testable: see line_follow_step().
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>
#include "track_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default control period in milliseconds (NFR1). */
#define LINE_FOLLOW_PERIOD_MS   5u

/**
 * Tuning / configuration for line following. Thresholds come from
 * ir_calibrate (assumption A2). "black" means an ADC reading on the
 * dark line; comparison direction is set by on_black().
 */
/*
 * Sensor layout: the two front sensors STRADDLE the line ("left" = the
 * left-of-line sensor, "center" = the right-of-line sensor; the "center"
 * name is kept so the API does not change). The line is narrower than
 * the sensor gap, so:
 *   both white            -> centred (NOT lost)
 *   left black            -> line under the left sensor  -> error > 0
 *   right("center") black -> line under the right sensor -> error < 0
 *   both black (held)     -> junction
 *   last_side             -> which sensor most recently saw black (kept
 *                            while both white), for bend recovery
 *   lost                  -> line data unusable (uncalibrated). An
 *                            optional time-based lost exists but is OFF
 *                            by default (lost_timeout_ms = 0) because
 *                            both-white also means "centred" on straights.
 */

/** Full-scale normalised "darkness" of one sensor (0 = white, max = black). */
#define LINE_DARK_MAX   511
/** Darkness at/above this means the line is near that sensor (resets the
    lost timer even before the sensor crosses its black threshold). */
#define LINE_NEAR_DARK  128

typedef struct {
    uint16_t left_threshold;    /* ADC level separating line vs background, Left */
    uint16_t center_threshold;  /* ADC level separating line vs background, Center */
    uint8_t  dark_is_high;      /* 1 if higher ADC == darker (sensor dependent) */
    uint16_t junction_hold_ms;  /* both-black must persist this long => junction */
    int16_t  error_gain;        /* scales the darkness difference into line error */
    /* Calibration means (from ir_cal_apply). Each sensor is normalised to
       0..LINE_DARK_MAX between its own white and black means, so sensors
       with different contrast steer equally. */
    uint16_t left_white,   left_black;
    uint16_t center_white, center_black;
    uint16_t lost_timeout_ms;   /* optional: both white & not near line this long => lost; 0 = off (default) */
} line_follow_cfg_t;

/** Internal state carried between 5 ms steps (debounce timers). */
typedef struct {
    uint32_t both_black_since_ms; /* 1-based start of both-black, 0 = not both-black */
    uint8_t  junction_latched;    /* 1 once the junction has been confirmed (held >= N ms) */
    uint32_t white_since_ms;      /* 1-based start of both-white-and-not-near, 0 = not */
    uint8_t  last_side;           /* LINE_SIDE_LEFT / LINE_SIDE_RIGHT: last sensor to see black */
} line_follow_state_t;

/** Raw analog sample pair for one step. */
typedef struct {
    uint16_t left_adc;
    uint16_t center_adc;
    uint32_t now_ms;
} line_sample_t;

/**
 * Pure compute step (NO RTOS, NO hardware) — the unit-test entry point.
 *
 * Given a raw sample and current config/state, produces the packed
 * line_err_t and updates state. Logic:
 *   - on_black(left), on_black(center) decide the discrete situation.
 *   - error = (darkness(left) - darkness(center)) * error_gain, clamped to
 *     10-bit. Centred (both white) => ~0; grows smoothly as the line moves
 *     under one sensor. Positive = line under the LEFT sensor.
 *   - both black held >= junction_hold_ms => junction flag (held high).
 *   - exactly one sensor black => last_side updated to that sensor;
 *     otherwise last_side is held (reported in every message).
 *   - optional: both white, neither near the line, for >= lost_timeout_ms
 *     (if non-zero)                       => lost_line flag, error = 0.
 *
 * @return packed line_err_t ready to send to line_err_q.
 */
line_err_t line_follow_step(const line_follow_cfg_t *cfg,
                            line_follow_state_t *state,
                            const line_sample_t *sample);

/** Reset the between-step state (call on entering RUN). */
void line_follow_reset(line_follow_state_t *state);

/** True if a reading counts as "on the black line" given the config. */
uint8_t line_follow_on_black(const line_follow_cfg_t *cfg,
                             uint16_t adc, uint8_t is_center);

/**
 * Normalise one reading to 0..LINE_DARK_MAX between that sensor's white
 * and black calibration means (either polarity). Clamped at both ends;
 * returns 0 if white == black (uncalibrated / no contrast).
 */
uint16_t line_follow_darkness(uint16_t adc, uint16_t white, uint16_t black);

/* --- RTOS-facing task entry (defined in line_follow.c, excluded from host build) --- */
#ifndef LINE_FOLLOW_HOST_TEST
#include "rtos_port.h"
/**
 * Line Follow task body: samples ADC every 5 ms, calls line_follow_step(),
 * sends to line_err_q (non-blocking), sets its heartbeat bit each loop.
 */
void line_follow_task(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */

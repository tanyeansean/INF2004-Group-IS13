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
typedef struct {
    uint16_t left_threshold;    /* ADC level separating line vs background, Left */
    uint16_t center_threshold;  /* ADC level separating line vs background, Center */
    uint8_t  dark_is_high;      /* 1 if higher ADC == darker (sensor dependent) */
    uint16_t junction_hold_ms;  /* both-black must persist this long => junction */
    int16_t  error_gain;        /* scales (left-center) difference into line error */
} line_follow_cfg_t;

/** Internal state carried between 5 ms steps (junction debounce timer). */
typedef struct {
    uint32_t both_black_since_ms; /* timestamp both-black began, 0 = not both-black */
    uint8_t  junction_latched;    /* 1 once the junction has been confirmed (held >= N ms) */
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
 *   - error = (left_adc - center_adc) * error_gain, clamped to 10-bit.
 *       centered on the followed edge => difference ~ 0.
 *   - both black held >= junction_hold_ms => junction flag (edge-latched).
 *   - both on background                 => lost_line flag, error = 0.
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

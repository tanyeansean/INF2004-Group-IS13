/**
 * @file line_follow.c
 * @brief Line Follow implementation (Buddy 3).
 *
 * The pure compute core (line_follow_step / line_follow_on_black) has no
 * RTOS or hardware dependency and is exercised by the host unit tests.
 * The task body is compiled only for the target (guarded by
 * LINE_FOLLOW_HOST_TEST being undefined).
 */
#include "line_follow.h"

/*
 * Sign convention for error:
 *   error = (left_adc - center_adc) scaled by error_gain, after removing
 *   the sensor polarity via dark_is_high (so "more black" always pushes
 *   the raw difference positive before scaling).
 *
 * We follow the LEFT edge of the line with Left + Center sensors:
 *   - Centered on the edge -> both sensors see a similar mix -> error ~ 0.
 *   - LEFT sees more black than Center -> error > 0.
 *   - CENTER sees more black than Left -> error < 0.
 * Mission Control interprets the sign; what matters here is that 0 is
 * centered and the magnitude grows with deviation.
 */

uint8_t line_follow_on_black(const line_follow_cfg_t *cfg,
                             uint16_t adc, uint8_t is_center)
{
    uint16_t thr = is_center ? cfg->center_threshold : cfg->left_threshold;
    if (cfg->dark_is_high) {
        return (uint8_t)(adc >= thr ? 1u : 0u);
    }
    return (uint8_t)(adc <= thr ? 1u : 0u);
}

void line_follow_reset(line_follow_state_t *state)
{
    state->both_black_since_ms = 0u;
    state->junction_latched = 0u;
}

static int16_t clamp_err(int32_t v)
{
    if (v < LINE_ERR_MIN) {
        return (int16_t)LINE_ERR_MIN;
    }
    if (v > LINE_ERR_MAX) {
        return (int16_t)LINE_ERR_MAX;
    }
    return (int16_t)v;
}

line_err_t line_follow_step(const line_follow_cfg_t *cfg,
                            line_follow_state_t *state,
                            const line_sample_t *sample)
{
    uint8_t left_black   = line_follow_on_black(cfg, sample->left_adc, 0u);
    uint8_t center_black = line_follow_on_black(cfg, sample->center_adc, 1u);

    uint8_t junction = 0u;
    uint8_t lost     = 0u;
    int16_t error    = 0;

    if (left_black && center_black) {
        /* Both black: candidate junction. Require it to hold N ms, then
           hold the junction flag HIGH for as long as both stay black, so
           Mission Control (polling at its own rate) cannot miss a brief
           one-sample pulse. */
        if (state->both_black_since_ms == 0u) {
            /* use 1-based start so a genuine t=0 still registers */
            state->both_black_since_ms = sample->now_ms + 1u;
        }
        {
            uint32_t began = state->both_black_since_ms - 1u;
            uint32_t held  = sample->now_ms - began;
            if (held >= cfg->junction_hold_ms) {
                junction = 1u;                 /* held high while both-black */
                state->junction_latched = 1u;  /* confirmed at least once */
            }
        }
        error = 0; /* straddling a crossing: no meaningful edge error */
    } else if (!left_black && !center_black) {
        /* Both on background: line lost. */
        lost = 1u;
        error = 0;
        state->both_black_since_ms = 0u;
        state->junction_latched = 0u;
    } else {
        /* Normal edge following: proportional difference. */
        int32_t diff = (int32_t)sample->left_adc - (int32_t)sample->center_adc;
        if (!cfg->dark_is_high) {
            diff = -diff; /* normalise so sign is sensor-polarity independent */
        }
        error = clamp_err(diff * (int32_t)cfg->error_gain);
        state->both_black_since_ms = 0u;
        state->junction_latched = 0u;
    }

    return line_err_pack(error, junction, lost);
}

/* ------------------------------------------------------------------ */
/* Target-only task body                                               */
/* ------------------------------------------------------------------ */
#ifndef LINE_FOLLOW_HOST_TEST
#include "rtos_port.h"
#include "ir_calibrate.h"

/* These are provided by the integration layer (queue/flag ids). */
extern rtos_queue_t g_line_err_q;
extern rtos_flag_t  g_hb_flag;
extern uint32_t     g_line_follow_hb_bit;

/* Hardware hooks implemented in the board layer (ADC reads). */
extern uint16_t board_read_left_adc(void);
extern uint16_t board_read_center_adc(void);

/*
 * READY-phase calibration hooks (board layer). The line task OWNS the
 * config and the calibration; nothing else writes it. During READY the
 * operator presents the line and the background in turn.
 *   board_line_ready()         -> 1 while still in the READY phase
 *   board_line_sample_is_line()-> 1 if the current view is the black line,
 *                                 0 if it is background (operator-signalled,
 *                                 e.g. via the start button or a fixed
 *                                 move-over-line-then-off sequence)
 */
extern uint8_t board_line_ready(void);
extern uint8_t board_line_sample_is_line(void);

void line_follow_task(void)
{
    /* cfg is OWNED by this task (no shared extern) -> single writer, and
       no other task can read a half-updated multi-field struct. */
    line_follow_cfg_t   cfg;
    line_follow_state_t state;
    ir_cal_accum_t      cal_left, cal_center;
    uint8_t             calibrated = 0u;

    /* Fixed tuning that is not derived from sensor data. */
    cfg.junction_hold_ms = 30u;
    cfg.error_gain       = 1;

    /* ---- READY: collect calibration samples, then compute thresholds ---- */
    ir_cal_reset(&cal_left);
    ir_cal_reset(&cal_center);
    while (board_line_ready()) {
        uint16_t l = board_read_left_adc();
        uint16_t c = board_read_center_adc();
        if (board_line_sample_is_line()) {
            ir_cal_add_line(&cal_left, l);
            ir_cal_add_line(&cal_center, c);
        } else {
            ir_cal_add_background(&cal_left, l);
            ir_cal_add_background(&cal_center, c);
        }
        (void)rtos_flag_set(g_hb_flag, g_line_follow_hb_bit); /* stay alive */
        rtos_delay_ms(LINE_FOLLOW_PERIOD_MS);
    }
    calibrated = ir_cal_apply(&cal_left, &cal_center, &cfg);

    line_follow_reset(&state);

    /* ---- RUN ---- */
    for (;;) {
        line_sample_t s;
        line_err_t    msg;

        /*
         * Refuse to follow the line uncalibrated. Report lost-line (error
         * 0) so Mission Control holds rather than steering on garbage.
         * This replaces the intent of the removed FAULT_IR_UNCAL bit with
         * a structural guard (preferred over an invented fault bit).
         */
        if (!calibrated) {
            msg = line_err_pack(0, 0u, 1u); /* lost-line, no error */
            (void)rtos_queue_send(g_line_err_q, &msg, sizeof(msg));
            (void)rtos_flag_set(g_hb_flag, g_line_follow_hb_bit);
            rtos_delay_ms(LINE_FOLLOW_PERIOD_MS);
            continue;
        }

        s.left_adc   = board_read_left_adc();
        s.center_adc = board_read_center_adc();
        s.now_ms     = rtos_now_ms();

        msg = line_follow_step(&cfg, &state, &s);

        /* zero-timeout send; if full, Mission Control just missed one sample */
        (void)rtos_queue_send(g_line_err_q, &msg, sizeof(msg));

        /* heartbeat: prove liveness to the Safety Monitor every loop */
        (void)rtos_flag_set(g_hb_flag, g_line_follow_hb_bit);

        rtos_delay_ms(LINE_FOLLOW_PERIOD_MS);
    }
}
#endif /* LINE_FOLLOW_HOST_TEST */

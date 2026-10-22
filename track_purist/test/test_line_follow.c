/**
 * @file test_line_follow.c
 * @brief Host unit tests for the line-follow core, packing, and calibration.
 */
#ifndef LINE_FOLLOW_HOST_TEST
#define LINE_FOLLOW_HOST_TEST  /* also set by the test CMake target */
#endif
#include "line_follow.h"
#include "ir_calibrate.h"
#include "test_util.h"

/* Config: dark_is_high = 1 (higher ADC = darker line), thresholds 2000. */
static line_follow_cfg_t make_cfg(void)
{
    line_follow_cfg_t c;
    c.left_threshold   = 2000u;
    c.center_threshold = 2000u;
    c.dark_is_high     = 1u;
    c.junction_hold_ms = 30u;
    c.error_gain       = 1;
    return c;
}

int main(void)
{
    printf("== line follow tests ==\n");

    /* --- packing round-trips --- */
    {
        line_err_t p;
        p = line_err_pack(100, 0, 0);
        CHECK_EQ_INT(line_err_get_error(p), 100, "pack/unpack +100");
        CHECK(!line_err_get_junction(p), "no junction flag");
        CHECK(!line_err_get_lost(p), "no lost flag");

        p = line_err_pack(-100, 0, 0);
        CHECK_EQ_INT(line_err_get_error(p), -100, "pack/unpack -100 (signed)");

        p = line_err_pack(511, 1, 0);
        CHECK_EQ_INT(line_err_get_error(p), 511, "pack/unpack max +511");
        CHECK(line_err_get_junction(p), "junction flag set");

        p = line_err_pack(-512, 0, 1);
        CHECK_EQ_INT(line_err_get_error(p), -512, "pack/unpack min -512");
        CHECK(line_err_get_lost(p), "lost flag set");

        /* clamping */
        p = line_err_pack(5000, 0, 0);
        CHECK_EQ_INT(line_err_get_error(p), 511, "over-range clamps to +511");
        p = line_err_pack(-5000, 0, 0);
        CHECK_EQ_INT(line_err_get_error(p), -512, "under-range clamps to -512");
    }

    /* --- on_black polarity --- */
    {
        line_follow_cfg_t c = make_cfg();
        CHECK(line_follow_on_black(&c, 2500, 0), "high adc = black (dark_is_high)");
        CHECK(!line_follow_on_black(&c, 1000, 0), "low adc = background");
    }

    /* --- step: centered => error ~ 0 (left == center) --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 2500, .center_adc = 2500, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        /* both black & centered: both_black path -> junction not yet, error 0 */
        CHECK_EQ_INT(line_err_get_error(r), 0, "centered both-black error 0");
    }

    /* --- step: edge following, left brighter than center => nonzero error --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        /* left on background (1000), center on black (2500): one black, one not */
        line_sample_t s = { .left_adc = 1000, .center_adc = 2500, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "not lost (one sensor on line)");
        CHECK(!line_err_get_junction(r), "not junction (only one black)");
        CHECK(line_err_get_error(r) != 0, "edge following gives nonzero error");
    }

    /* --- step: both background => lost line --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 800, .center_adc = 800, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        CHECK(line_err_get_lost(r), "both background => lost line");
        CHECK_EQ_INT(line_err_get_error(r), 0, "lost => error 0");
    }

    /* --- step: junction only after both-black held >= N ms, once --- */
    {
        line_follow_cfg_t c = make_cfg(); /* hold 30 ms */
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 2500, .center_adc = 2500, .now_ms = 0 };
        line_err_t r;

        s.now_ms = 0;  r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_junction(r), "t=0 both-black: no junction yet");

        s.now_ms = 20; r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_junction(r), "t=20ms: still below hold");

        s.now_ms = 35; r = line_follow_step(&c, &st, &s);
        CHECK(line_err_get_junction(r), "t=35ms: junction fires (>=30ms)");

        s.now_ms = 40; r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_junction(r), "junction latched: fires only once");
    }

    /* --- calibration derives threshold midpoint + polarity --- */
    {
        ir_cal_accum_t left, center;
        line_follow_cfg_t cfg;
        uint8_t ok;

        ir_cal_reset(&left);
        ir_cal_reset(&center);
        /* line (dark) reads high ~3000, background ~1000 */
        ir_cal_add_line(&left, 3000); ir_cal_add_line(&left, 3000);
        ir_cal_add_background(&left, 1000); ir_cal_add_background(&left, 1000);
        ir_cal_add_line(&center, 2800); ir_cal_add_line(&center, 3200);
        ir_cal_add_background(&center, 900); ir_cal_add_background(&center, 1100);

        ok = ir_cal_apply(&left, &center, &cfg);
        CHECK(ok, "calibration applies");
        CHECK_EQ_INT(cfg.left_threshold, 2000, "left threshold = midpoint 2000");
        CHECK_EQ_INT(cfg.center_threshold, 2000, "center threshold = midpoint 2000");
        CHECK(cfg.dark_is_high, "dark_is_high derived (line > background)");
    }

    /* --- calibration fails with no background samples --- */
    {
        ir_cal_accum_t a; uint16_t thr;
        ir_cal_reset(&a);
        ir_cal_add_line(&a, 3000);
        CHECK(!ir_cal_compute(&a, &thr), "no background => compute fails");
    }

    return TEST_SUMMARY();
}

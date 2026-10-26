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

/* Config: dark_is_high = 1 (higher ADC = darker line), thresholds 2000,
   white mean 1000, black mean 3000 for both sensors, lost after 1000 ms. */
static line_follow_cfg_t make_cfg(void)
{
    line_follow_cfg_t c;
    c.left_threshold   = 2000u;
    c.center_threshold = 2000u;
    c.dark_is_high     = 1u;
    c.junction_hold_ms = 30u;
    c.error_gain       = 1;
    c.left_white   = 1000u; c.left_black   = 3000u;
    c.center_white = 1000u; c.center_black = 3000u;
    c.lost_timeout_ms  = 1000u;
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

        /* last_side bit does not disturb other fields */
        p = line_err_with_last_side(line_err_pack(-300, 1, 1), LINE_SIDE_RIGHT);
        CHECK_EQ_INT(line_err_get_error(p), -300, "last_side keeps error");
        CHECK(line_err_get_junction(p) && line_err_get_lost(p), "last_side keeps flags");
        CHECK_EQ_INT(line_err_get_last_side(p), LINE_SIDE_RIGHT, "last_side RIGHT round-trip");
        p = line_err_with_last_side(p, LINE_SIDE_LEFT);
        CHECK_EQ_INT(line_err_get_last_side(p), LINE_SIDE_LEFT, "last_side cleared to LEFT");

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

    /* --- step: both black => error 0 (junction path) --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 2500, .center_adc = 2500, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_error(r), 0, "both-black error 0");
    }

    /* --- straddle: both white = CENTRED, not lost --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 1000, .center_adc = 1000, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "both white at t=0 => centred, NOT lost");
        CHECK_EQ_INT(line_err_get_error(r), 0, "centred => error 0");
        CHECK(!line_err_get_junction(r), "centred => no junction");
    }

    /* --- one sensor on the line => nonzero error, not lost/junction --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 1000, .center_adc = 2500, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "not lost (one sensor on line)");
        CHECK(!line_err_get_junction(r), "not junction (only one black)");
        CHECK(line_err_get_error(r) != 0, "one sensor on line gives nonzero error");
    }

    /* --- sign convention --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st;
        line_err_t r;

        line_follow_reset(&st);
        {
            line_sample_t s = { .left_adc = 3000, .center_adc = 1000, .now_ms = 0 };
            r = line_follow_step(&c, &st, &s);
        }
        CHECK_EQ_INT(line_err_get_error(r), 511, "line fully under LEFT => +511");

        line_follow_reset(&st);
        {
            line_sample_t s = { .left_adc = 1000, .center_adc = 3000, .now_ms = 0 };
            r = line_follow_step(&c, &st, &s);
        }
        CHECK(line_err_get_error(r) <= -511, "line fully under RIGHT => -511");
    }

    /* --- proportional: half-dark left gives about half error --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 2000, .center_adc = 1000, .now_ms = 0 };
        line_err_t r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_error(r), 255, "half-dark left => error 255");
    }

    /* --- normalisation: different contrast sensors steer equally --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st;
        line_err_t rl, rr;
        c.left_white = 1800u; c.left_black = 2400u;   /* weak sensor */
        c.left_threshold = 2100u;
        c.center_white = 150u; c.center_black = 2300u; /* strong sensor */
        c.center_threshold = 1225u;

        line_follow_reset(&st);
        { line_sample_t s = { .left_adc = 2400, .center_adc = 150, .now_ms = 0 };
          rl = line_follow_step(&c, &st, &s); }
        line_follow_reset(&st);
        { line_sample_t s = { .left_adc = 1800, .center_adc = 2300, .now_ms = 0 };
          rr = line_follow_step(&c, &st, &s); }
        CHECK_EQ_INT(line_err_get_error(rl), 511, "weak left sensor on line => +511");
        CHECK_EQ_INT(line_err_get_error(rr), -511, "strong right sensor on line => -511");
    }

    /* --- darkness helper, inverted polarity --- */
    {
        CHECK_EQ_INT(line_follow_darkness(3000, 3000, 1000), 0,   "inverted: white reads 0");
        CHECK_EQ_INT(line_follow_darkness(1000, 3000, 1000), 511, "inverted: black reads 511");
        CHECK_EQ_INT(line_follow_darkness(500, 1000, 1000), 0,    "no contrast => 0");
    }

    /* --- lost only after both white for lost_timeout_ms --- */
    {
        line_follow_cfg_t c = make_cfg();  /* 1000 ms */
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 1000, .center_adc = 1000, .now_ms = 0 };
        line_err_t r;

        r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "t=0 both white: not lost");
        s.now_ms = 999; r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "t=999 both white: not lost yet");
        s.now_ms = 1000; r = line_follow_step(&c, &st, &s);
        CHECK(line_err_get_lost(r), "t=1000 both white: lost");
        CHECK_EQ_INT(line_err_get_error(r), 0, "lost => error 0");

        /* seeing the line again clears lost and restarts the timer */
        s.now_ms = 1005; s.left_adc = 2500;
        r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "line seen again => not lost");
        s.now_ms = 1010; s.left_adc = 1000;
        r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "back to centred => timer restarted, not lost");
    }

    /* --- near-dark (below threshold) also keeps lost timer reset --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 1000, .center_adc = 1000, .now_ms = 0 };
        line_err_t r;
        (void)line_follow_step(&c, &st, &s);
        s.now_ms = 900; s.left_adc = 1600;   /* darkness ~153 >= near, still white */
        (void)line_follow_step(&c, &st, &s);
        s.now_ms = 1500; s.left_adc = 1000;
        r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "near-line glimpse restarts lost timer");
    }

    /* --- last_side: remembers the last sensor to see black --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 1000, .center_adc = 1000, .now_ms = 0 };
        line_err_t r;

        r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_last_side(r), LINE_SIDE_LEFT, "last_side defaults to LEFT");

        s.now_ms = 5; s.center_adc = 2500;          /* right sensor sees line */
        r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_last_side(r), LINE_SIDE_RIGHT, "right black => last_side RIGHT");
        CHECK(r.packed & LINE_LAST_SIDE_BIT, "last_side packed in bit 12");

        s.now_ms = 10; s.center_adc = 1000;         /* both white again */
        r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_last_side(r), LINE_SIDE_RIGHT, "both white => last_side held RIGHT");
        CHECK_EQ_INT(line_err_get_error(r), 0, "both white => error 0");

        s.now_ms = 15; s.left_adc = 2500; s.center_adc = 2500; /* junction */
        r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_last_side(r), LINE_SIDE_RIGHT, "both black => last_side held");

        s.now_ms = 20; s.center_adc = 1000;          /* left only */
        r = line_follow_step(&c, &st, &s);
        CHECK_EQ_INT(line_err_get_last_side(r), LINE_SIDE_LEFT, "left black => last_side LEFT");
        CHECK(!(r.packed & LINE_LAST_SIDE_BIT), "bit 12 clear for LEFT");
        CHECK_EQ_INT(r.packed & 0xE000u, 0, "bits 13..15 stay 0");
    }

    /* --- default (lost_timeout_ms = 0): long straight never reports lost --- */
    {
        line_follow_cfg_t c = make_cfg();
        line_follow_state_t st; line_follow_reset(&st);
        line_sample_t s = { .left_adc = 1000, .center_adc = 1000, .now_ms = 0 };
        line_err_t r;
        c.lost_timeout_ms = 0u;
        (void)line_follow_step(&c, &st, &s);
        s.now_ms = 60000; r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_lost(r), "timeout 0 => 60 s centred, never lost");
    }

    /* --- step: junction only after both-black held >= N ms, once --- */
    /* --- junction: fires after both-black held >= N ms, then HELD high --- */
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

        /* held high on every subsequent step while both stay black */
        s.now_ms = 40; r = line_follow_step(&c, &st, &s);
        CHECK(line_err_get_junction(r), "t=40ms: junction still held high");

        s.now_ms = 60; r = line_follow_step(&c, &st, &s);
        CHECK(line_err_get_junction(r), "t=60ms: junction still held high");

        /* leaving both-black drops the flag (edge following resumes) */
        s.now_ms = 65; s.left_adc = 1000; s.center_adc = 2500;
        r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_junction(r), "junction clears when both-black ends");

        /* re-entering both-black must re-arm the hold timer (not instant) */
        s.now_ms = 70; s.left_adc = 2500; s.center_adc = 2500;
        r = line_follow_step(&c, &st, &s);
        CHECK(!line_err_get_junction(r), "re-entry: hold timer restarts, no junction yet");
        s.now_ms = 105; r = line_follow_step(&c, &st, &s);
        CHECK(line_err_get_junction(r), "re-entry: junction fires again after hold");
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
        CHECK_EQ_INT(cfg.left_white, 1000, "left white mean stored");
        CHECK_EQ_INT(cfg.left_black, 3000, "left black mean stored");
        CHECK_EQ_INT(cfg.center_white, 1000, "center white mean stored");
        CHECK_EQ_INT(cfg.center_black, 3000, "center black mean stored");
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

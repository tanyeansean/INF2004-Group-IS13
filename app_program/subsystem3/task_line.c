/*
 *	task_line.c
 *	Subsystem 3 -- Line Follow task: calibration at boot, then every 5 ms
 *	reads the front IR pair, runs line_follow_step() and sends the packed
 *	result to Mission Control (line_err_q, zero-timeout send).
 *
 *	Same procedure and numbers as integrated_car/main.c (calibrate()):
 *	3 s countdown, 150 samples x 10 ms on BLACK, then the same on WHITE,
 *	junction = both black held 30 ms, error gain 1, time-based lost OFF.
 *	The task never drives the motors (Week 6 rule).
 */

#include "subsystem3.h"
#include "drv_track.h"
#include "line_follow.h"
#include "ir_calibrate.h"
#include "../subsystem1/subsystem1.h"	/* flg_hb, HB_LINE_FOLLOW */
#include "../subsystem2/subsystem2.h"	/* IF-01: mbf_line_err */

#define CAL_COUNTDOWN_S		3
#define CAL_SAMPLES		150	/* x 10 ms = 1.5 s per phase */
#define CAL_MIN_CONTRAST	200U

EXPORT volatile UW	s3_calibrated = 0U;

LOCAL UW	s3_line_dropped = 0U;	/* line_err_q full (counted, G3 style) */

LOCAL void hb(void)
{
	(void)tk_set_flg(flg_hb, HB_LINE_FOLLOW);
}

/* Sleep ms, setting the heartbeat at least every 20 ms. */
LOCAL void wait_ms(UW ms)
{
	while (ms > 0U) {
		UW step = (ms > 20U) ? 20U : ms;
		hb();
		tk_dly_tsk((RELTIM)step);
		ms -= step;
	}
}

LOCAL void countdown(const char *msg)
{
	INT s;

	tm_printf((UB *)"%s\n", msg);
	for (s = CAL_COUNTDOWN_S; s > 0; s--) {
		tm_printf((UB *)"  starting in %d...\n", s);
		wait_ms(1000U);
	}
	tm_printf((UB *)"  SAMPLING - hold still (1.5 s)\n");
}

LOCAL UW udiff(UW a, UW b) { return (a > b) ? (a - b) : (b - a); }

/* Returns TRUE if calibration succeeded (cfg filled in). */
LOCAL BOOL calibrate(line_follow_cfg_t *cfg)
{
	ir_cal_accum_t	cl, cr;
	UW		lb, lw, rb, rw;
	INT		i;

	ir_cal_reset(&cl);
	ir_cal_reset(&cr);

	countdown("CALIBRATE 1/2: put BOTH front sensors over the BLACK LINE");
	for (i = 0; i < CAL_SAMPLES; i++) {
		ir_cal_add_line(&cl, drv_track_read_left());
		ir_cal_add_line(&cr, drv_track_read_right());
		wait_ms(10U);
	}
	countdown("CALIBRATE 2/2: put BOTH front sensors over WHITE");
	for (i = 0; i < CAL_SAMPLES; i++) {
		ir_cal_add_background(&cl, drv_track_read_left());
		ir_cal_add_background(&cr, drv_track_read_right());
		wait_ms(10U);
	}

	cfg->junction_hold_ms = 30U;
	cfg->error_gain       = 1;
	cfg->lost_timeout_ms  = 0U;
	if (!ir_cal_apply(&cl, &cr, cfg)) {
		return FALSE;
	}

	lb = cl.line_sum / cl.line_count;  lw = cl.bg_sum / cl.bg_count;
	rb = cr.line_sum / cr.line_count;  rw = cr.bg_sum / cr.bg_count;
	tm_printf((UB *)"L black=%u white=%u | R black=%u white=%u\n", lb, lw, rb, rw);
	if ((udiff(lb, lw) < CAL_MIN_CONTRAST) || (udiff(rb, rw) < CAL_MIN_CONTRAST)) {
		tm_printf((UB *)"WARNING: low contrast on a sensor - check height / wiring\n");
	}
	return TRUE;
}

EXPORT void line_task(INT stacd, void *exinf)
{
	line_follow_cfg_t	cfg;
	line_follow_state_t	state;
	UW			next_ms;
	ER			er;

	(void)stacd; (void)exinf;

	er = drv_track_adc_open();
	CHECK(er, "adc open");

	s3_wait_console();
	wait_ms(500U);		/* after Mission Control's banner (300 ms) */
	tm_printf((UB *)"Commands work only AFTER calibration (when READY is printed).\n");

	if ((er < E_OK) || !calibrate(&cfg)) {
		tm_printf((UB *)"CALIBRATION FAILED - halting (motors off)\n");
		for (;;) {
			wait_ms(100U);		/* stay alive, never report a line */
		}
	}

	line_follow_reset(&state);
	s3_calibrated = 1U;
	tm_printf((UB *)"READY. Send 'start' over MQTT or press GP20. GP20 again = stop.\n");

	next_ms = s3_now_ms();
	for (;;) {
		line_sample_t	s;
		line_err_t	le;
		UINT		ptn = 0U;
		W		delay;

		/* IF-03: Mission Control asked for a fresh start (START). */
		if (tk_wai_flg(flg_s3, S3_FLG_LINE_RESET, TWF_ORW, &ptn, TMO_POL) == E_OK) {
			line_follow_reset(&state);
			(void)tk_clr_flg(flg_s3, ~S3_FLG_LINE_RESET);
		}

		s.left_adc   = drv_track_read_left();
		s.center_adc = drv_track_read_right();
		s.now_ms     = s3_now_ms();
		le = line_follow_step(&cfg, &state, &s);

		if (tk_snd_mbf(mbf_line_err, &le, (INT)sizeof(le), TMO_POL) != E_OK) {
			s3_line_dropped++;
		}
		hb();

		/* Fixed 5 ms period (catch up after a stall, never burst). */
		next_ms += LINE_FOLLOW_PERIOD_MS;
		delay = (W)(next_ms - s3_now_ms());
		if (delay > 0) {
			tk_dly_tsk((RELTIM)delay);
		} else if (delay < -(W)LINE_FOLLOW_PERIOD_MS) {
			next_ms = s3_now_ms();
		}
	}
}

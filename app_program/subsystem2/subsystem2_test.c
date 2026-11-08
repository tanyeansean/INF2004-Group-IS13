/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "../subsystem3/subsystem3.h"	/* s3_wait_console() */
#include "../test_report.h"

#if APP_TEST

#define S2_ENC_SPIN_S		15	/* seconds to turn one wheel by hand */
#define S2_ENC_POLL_MS		10	/* level sampling, independent of the ISR */
#define S2_ENC_MIN_COUNTS	20	/* less than this: no real signal   */
#define S2_ENC_CAL_WAIT_S	60	/* wait for READY (line calibration) */
#define S2_ENC_PAUSE_MS		3000	/* read this to start, hands off    */

/* Manual: the tester turns one wheel forward by exactly one revolution.
   Once a second prints the interrupt counts, the raw A/B levels and how
   often the polled levels changed (pin activity seen WITHOUT the
   interrupt).  Returns the count change of the wheel under test
   (*p_other: change of the other wheel, *p_pin_chg: polled changes). */
LOCAL W enc_spin(const char *wheel, BOOL left, W *p_other, UW *p_pin_chg)
{
	W	l0, r0, l, r;
	UW	ab_l, ab_r, chg_l = 0U, chg_r = 0U;
	UW	prev_l, prev_r;
	INT	s, k;

	tm_printf((UB *)"\n[S2] ===== %s WHEEL: let go of the other wheel. Turn the %s wheel"
		  " FORWARD exactly 1 full revolution (use your mark), then STOP. %d s =====\n",
		  wheel, wheel, S2_ENC_SPIN_S);
	tk_dly_tsk(S2_ENC_PAUSE_MS);	/* a wheel still coasting must not count */
	drv_encoder_read(&l0, &r0);
	drv_encoder_levels(&prev_l, &prev_r);
	for (s = S2_ENC_SPIN_S; s > 0; s--) {
		for (k = 0; k < (1000 / S2_ENC_POLL_MS); k++) {
			tk_dly_tsk(S2_ENC_POLL_MS);
			drv_encoder_levels(&ab_l, &ab_r);
			if (ab_l != prev_l) { chg_l++; prev_l = ab_l; }
			if (ab_r != prev_r) { chg_r++; prev_r = ab_r; }
		}
		drv_encoder_read(&l, &r);
		tm_printf((UB *)"[S2] %2d s left | count L %6d R %6d | pins L A%u B%u (%u changes)"
			  " R A%u B%u (%u changes)\n",
			  s - 1, (INT)(l - l0), (INT)(r - r0),
			  (UINT)(ab_l >> 1), (UINT)(ab_l & 1U), (UINT)chg_l,
			  (UINT)(ab_r >> 1), (UINT)(ab_r & 1U), (UINT)chg_r);
	}
	*p_pin_chg = left ? chg_l : chg_r;
	*p_other   = left ? (r - r0) : (l - l0);
	return left ? (l - l0) : (r - r0);
}

LOCAL void enc_report(const char *wheel, W d, UW pin_chg, const char *flag)
{
	if (d >= S2_ENC_MIN_COUNTS) {
		tm_printf((UB *)"[S2] %s wheel: %d counts per revolution (%u um per count)\n",
			  wheel, (INT)d, (UINT)((UW)ENC_WHEEL_CIRC_UM / (UW)d));
	} else if (d <= -S2_ENC_MIN_COUNTS) {
		tm_printf((UB *)"[S2] %s wheel counts DOWN going forward (%d):"
			  " set %s to 1 in drv_encoder.h\n", wheel, (INT)d, flag);
	} else if (pin_chg > 0U) {
		tm_printf((UB *)"[S2] %s wheel: pins changed %u times but only %d counts:"
			  " interrupt problem (software)\n", wheel, (UINT)pin_chg, (INT)d);
	} else {
		tm_printf((UB *)"[S2] %s wheel: pins NEVER changed: encoder power, ground"
			  " or A/B wires (hardware)\n", wheel);
	}
}

EXPORT void subsystem2_test(void)
{
	UW	m;
	W	d, other;
	UW	edges_l, edges_r, bad_l0, bad_r0, bad_l, bad_r;
	UW	ab_l, ab_r, pin_chg;
	INT	k;

	TEST_INIT();

	/* S2-U01: wheel message packs both signs and clamps */
	m = s2_wheels_pack(-255, 300);
	TEST_CHECK("S2-U01", s2_wheel_get(m, 0U) == -255 &&
		   s2_wheel_get(m, S2_WHEEL_BITS) == 255 && (m >> 18) == 0U,
		   "wheel PWM pair packs into 18 bits, clamped to +/-255");

	/* S2-U02: the stored map uses only bits 0-20 (20 map bits + valid) */
	TEST_CHECK("S2-U02", (nav_map_snapshot(&g_nav_map) >> 21) == 0U,
		   "barcode map snapshot fits 21 bits");

	/* S2-U03 .. S2-U05: wheel encoders, turned by hand (manual).  Start
	   after the line calibration prompts (READY), so they do not mix. */
	s3_wait_console();
	for (k = 0; (k < S2_ENC_CAL_WAIT_S) && !s3_is_calibrated(); k++) {
		tk_dly_tsk(1000);
	}
	tk_dly_tsk(2000);	/* let the READY / safety banners print first */
	drv_encoder_levels(&ab_l, &ab_r);
	tm_printf((UB *)"\n[S2] encoder test. Lift the car so both wheels turn freely.\n"
		  "[S2] pins now: L GP16=%u GP17=%u | R GP0=%u GP1=%u\n",
		  (UINT)(ab_l >> 1), (UINT)(ab_l & 1U), (UINT)(ab_r >> 1), (UINT)(ab_r & 1U));
	tk_dly_tsk(3000);
	drv_encoder_diag(&edges_l, &edges_r, &bad_l0, &bad_r0);

	d = enc_spin("LEFT", TRUE, &other, &pin_chg);
	enc_report("LEFT", d, pin_chg, "ENC_INVERT_LEFT");
	TEST_CHECK("S2-U03", d >= S2_ENC_MIN_COUNTS && (other < S2_ENC_MIN_COUNTS) &&
		   (other > -S2_ENC_MIN_COUNTS),
		   "left encoder (GP16/17) counts up going forward, right stays still");

#if ENC_RIGHT_ENABLED
	d = enc_spin("RIGHT", FALSE, &other, &pin_chg);
	enc_report("RIGHT", d, pin_chg, "ENC_INVERT_RIGHT");
	TEST_CHECK("S2-U04", d >= S2_ENC_MIN_COUNTS && (other < S2_ENC_MIN_COUNTS) &&
		   (other > -S2_ENC_MIN_COUNTS),
		   "right encoder (GP0/1) counts up going forward, left stays still");
#else
	TEST_CHECK("S2-U04", FALSE,
		   "right encoder off: GP0/GP1 are UART0 in this build (CRASH_REPORT/CONSOLE=uart)");
#endif

	drv_encoder_diag(&edges_l, &edges_r, &bad_l, &bad_r);
	tm_printf((UB *)"[S2] invalid transitions: L %u, R %u\n",
		  (UINT)(bad_l - bad_l0), (UINT)(bad_r - bad_r0));
	TEST_CHECK("S2-U05", (bad_l == bad_l0) && (bad_r == bad_r0),
		   "no invalid A/B transitions (no missed edges) while turning");

	/* S2-U06: counts -> distance, both signs, whole and part revolutions */
	TEST_CHECK("S2-U06",
		   (s2_counts_to_um(ENC_COUNTS_PER_REV) == ENC_WHEEL_CIRC_UM) &&
		   (s2_counts_to_um(-ENC_COUNTS_PER_REV) == -ENC_WHEEL_CIRC_UM) &&
		   (s2_counts_to_um(10 * ENC_COUNTS_PER_REV + (ENC_COUNTS_PER_REV / 2)) ==
		    (10 * ENC_WHEEL_CIRC_UM) + ((ENC_COUNTS_PER_REV / 2) * ENC_WHEEL_CIRC_UM) / ENC_COUNTS_PER_REV) &&
		   (s2_counts_to_um(0) == 0),
		   "encoder counts convert to micrometres (1 rev = wheel circumference)");

	/* S2-U07: motor_task publishes odometry every 20-30 ms; wheels at rest
	   (hands off the wheels) read 0 mm/s */
	{
		s2_odom_t	o1, o2;

		tm_printf((UB *)"[S2] S2-U07: hands off both wheels (1 s)\n");
		tk_dly_tsk(1000);
		s2_odom_get(&o1);
		tk_dly_tsk(300);
		s2_odom_get(&o2);
		tm_printf((UB *)"[S2] odometry: %u samples in 300 ms, window %u us,"
			  " L %d mm %d mm/s, R %d mm %d mm/s\n",
			  (UINT)(o2.updates - o1.updates), (UINT)o2.window_us,
			  (INT)o2.dist_mm_l, (INT)o2.speed_mms_l,
			  (INT)o2.dist_mm_r, (INT)o2.speed_mms_r);
		TEST_CHECK("S2-U07", ((o2.updates - o1.updates) >= 8U) &&
			   (o2.window_us >= (S2_ODOM_PERIOD_US - S2_ODOM_JITTER_US)) &&
			   (o2.window_us <= 40000U) &&
			   (o2.speed_mms_l == 0) && (o2.speed_mms_r == 0),
			   "odometry updated about every 20 ms, 0 mm/s with the wheels still");
	}

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

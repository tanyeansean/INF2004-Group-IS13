/*
 *	task_mission.c
 *	Subsystem 2 -- Mission Control task.  Port of the car logic in
 *	integrated_car/main.c v0.8 (same rules, same numbers):
 *
 *	  both white          -> straight          PWM 36 / 36
 *	  both white, <= 80 ms since one sensor last saw the line
 *	                      -> light steer, +-16 (bend recovery)
 *	  one sensor black    -> steer up to +-36, scaled by err/14
 *	  both black          -> junction (rising edge): stop, then the barcode
 *	                         move on wheel encoders, a rescan, or stay
 *	                         stopped (see "Junction moves" below)
 *	  lost (uncalibrated) -> stop
 *
 *	Runs once per line_err_q message (every 5 ms, like the bare-metal
 *	loop) and at least every 20 ms (timeout).  Inputs: line_err_q,
 *	barcode_q, cmd_q (MQTT), the GP20 button.  Output: wheel commands to
 *	Motor Control, data for Telemetry (status_q), the 2 Hz status line.
 *	Serial commands (S, X, 1, 2, ABCD, #Z) are not ported: the template's
 *	USB console has no input (tm_getchar is not implemented); use MQTT or
 *	GP20.
 */

#include <sys/machine.h>
#include "subsystem2.h"
#include "../subsystem1/subsystem1.h"	/* flg_hb, IF-05 s1_post_*, link */
#include "../subsystem3/subsystem3.h"	/* calibration, IF-03 line reset */
#include "../subsystem4/subsystem4.h"	/* s4_new_run(): highest hump per run */
#if APP_WITH_S5
#include "../subsystem5/subsystem5.h"	/* flg_avoid, AVOID_EVT_* (IF-09/10 replies) */
#endif

/* ---- Tuning (open loop: no encoders yet - same as integrated_car) ---- */
#ifndef STEERING_ENABLED
#define STEERING_ENABLED	1
#endif
/* 36 -> 45 (Jin, 13 Nov): at 36 Jin's car stalled 2 of 3 runs (once on the
   black ramp, once on flat; wheels 30-55 mm/s). At 45: 5 of 5 runs over the
   ramp, plus 13 more runs later. Obstacle moves use AV_DRIVE_PWM /
   AV_TURN_PWM, so they don't change. Logs: testing/subsystem4/logs/2026-11-13_*. */
#define BASE_PWM_M1		45
#define BASE_PWM_M2		45	/* same as M1: no trim, both wheels equal */
#define STEER_MAX		36	/* clamp on |steer| = max one-wheel offset */
#define STEER_ERR_DIV		14	/* steer = err/14; err is +-511 -> +-36 */
#define STEER_HOLD_LEVEL	16	/* light steer kept while the line sits
					   between the sensors, inside the hold */
#define STEER_HOLD_MS		80U	/* how long that hold lasts after the
					   line was last under a sensor; 0 = none */
#ifndef STEER_INVERT
#define STEER_INVERT		0	/* verified on car: right black -> turn right */
#endif
#ifndef JUNCTION_STOPS
#define JUNCTION_STOPS		0	/* 0 = barcode move at a junction (below);
					   1 = stop at every junction, no move */
#endif
#ifndef PWM_AT_SPEED_MAX
#define PWM_AT_SPEED_MAX	85	/* PWM for "speed 20" (not yet measured) */
#endif
#define MOTOR_LEFT_IS_M1	1

/* ---- Junction moves (wheel encoders) ----
 * Junction = rising edge of "both front sensors black". The car stops,
 * then by the pending barcode letter and the MQTT map:
 *   LEFT / RIGHT : forward JN_CREEP_MM (wheels onto the crossing), spin 90
 *   FORWARD      : forward JN_CREEP_MM, then line following again
 *   U-TURN       : forward JN_CREEP_MM, spin 180 (to the right)
 *   letter not in the map, or no barcode read at all : stop (wait for START)
 *   only an unreadable barcode since the last junction : reverse past the
 *     barcode and line-follow back over it (a rescan), up to JN_RESCAN_MAX
 *     times, then stop.
 * Every move ends on encoder distance; a stalled wheel or the time cap
 * stops the car (ROBOT_ERROR). */
#define JN_CREEP_MM		120	/* front IR sensors to the wheel axle (measured) */
#define JN_DRIVE_PWM		45	/* creep and rescan reverse (= line-follow base) */
#define JN_TURN_PWM		80	/* spins: same as the obstacle spins (AV_TURN_PWM) */
#define JN_TRACK_MM		110U	/* tyre centre to tyre centre (Jin, 13 Nov) */
#define JN_PI_360_E7		87266U	/* pi / 360 x 1e7 */
/* arc per wheel for an on-the-spot spin, um: 90 -> 86393, 180 -> 172786 */
#define JN_ARC_UM(deg)		((W)(((UW)(deg) * JN_TRACK_MM * JN_PI_360_E7) / 10000U))
#define JN_PAUSE_MS		200U	/* still before and after a move */
#define JN_STALL_MS		500U	/* a wheel with no progress this long = stalled */
#define JN_STALL_COUNTS		12	/* about 1 mm of wheel travel */
#define JN_MOVE_CAP_MS		8000U	/* backstop for one move */
#define JN_HOLD_COUNTS_PER_PWM	12	/* straight moves: wheel-difference correction */
#define JN_HOLD_MAX_PWM		15
#define JN_RESCAN_MAX		3U	/* rescans per junction */
#define JN_RESCAN_MARGIN_MM	60	/* run-up: reverse this far past the first bar */
#define JN_RESCAN_DEFAULT_MM	250	/* reverse when the barcode position is unknown */
#define JN_RESCAN_MAX_MM	600	/* never reverse further than this */
#define JN_BAR_MIN_EDGES	4U	/* fewer edges = flicker, not a barcode attempt.
					   10 -> 4 (Jin, 19 Nov): at speed the reader
					   caught only 2-4 of 30 edges of a real barcode,
					   so it was never marked. 4 = two bars, so the
					   junction's own cross line (one bar, 2 edges)
					   does not count. */
#define JN_RESCAN_ARRIVE_MM	30	/* rescan: junctions count again this close to
					   the one that started it (Jin, 19 Nov) */
#define JN_BAR_GAP_MS		500U	/* GP7 quiet this long: next edge = new pass */

#define S2_LINE_WAIT_MS		20
#define S2_STATUS_MS		500U
#define S2_TELEM_LINE_EVERY	4U	/* line data to Telemetry every 20 ms */

/* GP20: Robo Pico user button, active low (internal pull-up). */
#define S2_PIN_BUTTON		20U
#define S2_BUTTON_STABLE_MS	50U
#define S2_PADS_BANK0		(0x4001C000U)
#define S2_PAD(pin)		(S2_PADS_BANK0 + 0x04U + (4U * (pin)))
#define S2_PAD_PDE		(1U << 2)
#define S2_PAD_PUE		(1U << 3)
#define REG_ALIAS_SET		(0x2000U)
#define REG_ALIAS_CLR		(0x3000U)

typedef enum {
	CAR_IDLE = 0, CAR_FOLLOW, CAR_MANEUVER,
	CAR_AVOID		/* subsystem 5 has the wheels: line steering is
				   off and only S5's motion_q moves are driven,
				   until S5 sends CLEAR (or FAULT) */
} car_mode_t;

/* Mission Control's state: this task only. */
LOCAL car_mode_t	g_mode = CAR_IDLE;
LOCAL H			g_base_m1 = BASE_PWM_M1;
LOCAL H			g_base_m2 = BASE_PWM_M2;
#if MOTOR_LEFT_IS_M1
#define BASE_LEFT	g_base_m1
#define BASE_RIGHT	g_base_m2
#else
#define BASE_LEFT	g_base_m2
#define BASE_RIGHT	g_base_m1
#endif
LOCAL H			g_out_l, g_out_r;
LOCAL UW		g_hold_until_ms;	/* end of the steer-hold window (0 = none) */
LOCAL UB		g_lb, g_rb;
LOCAL nav_state_t	g_nav;

/* Junction move state (CAR_MANEUVER): this task only. */
typedef enum {
	JN_NONE = 0, JN_PAUSE_IN, JN_CREEP, JN_TURN, JN_REVERSE, JN_PAUSE_OUT
} jn_phase_t;
LOCAL jn_phase_t	g_jn_phase = JN_NONE;
LOCAL UB		g_jn_move;		/* BARCMD_* */
LOCAL BOOL		g_jn_rescan;		/* this move is a rescan reverse */
LOCAL W			g_jn_rescan_um;		/* how far that reverse goes */
LOCAL UINT		g_jn_rescans;		/* rescans done at this junction */
LOCAL UB		g_jn_prev;		/* junction flag, previous message */
LOCAL UW		g_jn_until_ms;		/* end of a pause */
LOCAL W			g_jn_target_um;		/* distance, or arc per wheel */
LOCAL W			g_jn_start_l, g_jn_start_r;
LOCAL BOOL		g_jn_fwd_l, g_jn_fwd_r;
LOCAL BOOL		g_jn_straight;
LOCAL H			g_jn_pwm;
LOCAL W			g_jn_hold;
LOCAL W			g_jn_best_l, g_jn_best_r;
LOCAL UW		g_jn_prog_ms_l, g_jn_prog_ms_r;
LOCAL UW		g_jn_begin_ms;

/* Barcode passes, from S3's queued-edge counter + the encoders. */
LOCAL UW		g_bar_last_pushed;
LOCAL UW		g_bar_last_edge_ms;
LOCAL UW		g_bar_pass_edges;	/* edges in the current pass */
LOCAL W			g_bar_pass_min_um;	/* rearmost odometer reading at an edge */
LOCAL BOOL		g_bar_pass_seen = FALSE;
LOCAL BOOL		g_bar_unreadable = FALSE; /* real INVALID since the last junction */
LOCAL BOOL		g_jn_ignoring = FALSE;	/* rescan: junctions ignored ... */
LOCAL W			g_jn_ignore_below_um;	/* ... until the odometer is back here */
LOCAL W			g_bar_unreadable_min_um;

LOCAL const char * const k_jn_move[BARCMD_COUNT] = { "LEFT", "RIGHT", "FORWARD", "U-TURN" };
LOCAL UW		g_motor_dropped = 0U;

LOCAL UW now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return (UW)t.lo;
}

LOCAL void motor_set(H left, H right)
{
	UW m = s2_wheels_pack(left, right);

	if (tk_snd_mbf(mbf_motor, &m, (INT)sizeof(m), TMO_POL) != E_OK) {
		g_motor_dropped++;
	}
}

LOCAL void car_stop(robot_state_t why)
{
	motor_set(0, 0);
#if APP_WITH_S5
	/* Leaving an avoid early (STOP, ESTOP, GP20, give-up): S5 is waiting
	   for a reply that will never come. Tell it, so it goes back to
	   MONITOR instead of timing out into a FAULT. */
	if (g_mode == CAR_AVOID) {
		(void)tk_set_flg(flg_avoid, AVOID_EVT_ABORT);
	}
#endif
	g_mode = CAR_IDLE;
	g_jn_phase = JN_NONE;
	g_hold_until_ms = 0U;
	s1_post_state(why);
}

/* Back to line following without the START checks (after an avoid). */
LOCAL void car_resume_follow(void)
{
	s3_request_line_reset();	/* = line_follow_reset(ls) in main.c */
	g_hold_until_ms = 0U;
	g_mode = CAR_FOLLOW;
	s1_post_state(ROBOT_LINE_FOLLOW);
}

LOCAL void car_start(void)
{
	if (!s3_is_calibrated()) {
		tm_printf((UB *)"[car] start ignored - still calibrating\n");
		return;
	}
	/* S4 (added 17 Nov by Jin): no start while the IMU is still calibrating
	   (the car must stand still for it) or while the car is lifted or
	   overturned (car test 17 Nov: a START was accepted upside down).
	   A dead IMU does not block the start (subsystem4.h, s4_start_ok). */
	if (!s4_start_ok()) {
		tm_printf((UB *)"[car] start ignored - IMU calibrating, or the car is off the floor."
			  " Put it down, hands off, wait for the [imu] line, press again.\n");
		return;
	}
#if APP_WITH_S5
	/* START also clears S5's latched fault: only Mission Control clears
	   fault bits. RESUME takes S5 out of its FAULT state. */
	(void)tk_clr_flg(flg_faults, ~(UINT)(FAULT_BIT_US_NO_ECHO | FAULT_BIT_NO_SAFE_RTE |
					      FAULT_BIT_US_TIMEOUT | FAULT_BIT_NO_LINE));
	if (s5_get_state() == S5_STATE_FAULT) {
		(void)tk_set_flg(flg_avoid, AVOID_EVT_RESUME);
	}
#endif
	/* A new run (S4, added 13 Nov by Jin): forget the highest hump, so
	   moving the car by hand before START can't count, and clear S4's
	   lifted / overturned bit (bit 2), which only Mission Control clears.
	   19 Nov (Jin): also clear the IMU fault (bit 5), so each run's
	   summary shows only that run's IMU drop-outs. S4 sets it again at
	   once if the sensor is still not answering. */
	(void)tk_clr_flg(flg_faults, ~(UINT)(FAULT_BIT_COLLISION | FAULT_BIT_IMU_FAULT));
	s4_new_run();
	g_jn_rescans = 0U;
	/* A barcode "pass" seen while the car was carried or placed is not a
	   barcode on this run (car test 19 Nov 19:55: it made the first
	   junction reverse a fixed 250 mm). Forget it (Jin, 19 Nov). */
	g_bar_unreadable = FALSE;
	g_bar_pass_seen  = FALSE;
	g_jn_ignoring    = FALSE;
	car_resume_follow();
	tm_printf((UB *)"[car] START (base PWM M1=%d M2=%d)\n", g_base_m1, g_base_m2);
}

/* =====================================================================
 *  Junction moves: stop, then creep / spin / rescan-reverse on encoders.
 * ===================================================================== */
LOCAL telem_move_t to_telem_move(UB barcmd);

/* Odometer: mean of both wheels' counts (left only without the right
   encoder), in um. + = forward. */
LOCAL W odo_um(void)
{
	W l, r;

	drv_encoder_read(&l, &r);
#if ENC_RIGHT_ENABLED
	return (s2_counts_to_um(l) + s2_counts_to_um(r)) / 2;
#else
	(void)r;
	return s2_counts_to_um(l);
#endif
}

LOCAL W jn_progress(W now, W start, BOOL fwd)
{
	W d = fwd ? (now - start) : (start - now);

	return (d < 0) ? 0 : d;
}

LOCAL BOOL jn_stalled(W prog, W *p_best, UW *p_ms, UW t)
{
	if (prog >= (*p_best + JN_STALL_COUNTS)) {
		*p_best = prog;
		*p_ms   = t;
		return FALSE;
	}
	return ((t - *p_ms) >= JN_STALL_MS) ? TRUE : FALSE;
}

/* Start one encoder move: constant PWM until target_um (distance, or the
   arc per wheel for a spin). */
LOCAL void jn_move(jn_phase_t phase, H left, H right, W target_um)
{
	drv_encoder_read(&g_jn_start_l, &g_jn_start_r);
	g_jn_phase     = phase;
	g_jn_fwd_l     = (left >= 0) ? TRUE : FALSE;
	g_jn_fwd_r     = (right >= 0) ? TRUE : FALSE;
	g_jn_straight  = (left == right) ? TRUE : FALSE;
	g_jn_pwm       = (left < 0) ? (H)-left : left;
	g_jn_hold      = 0;
	g_jn_target_um = target_um;
	g_jn_begin_ms  = now_ms();
	g_jn_best_l    = 0;
	g_jn_best_r    = 0;
	g_jn_prog_ms_l = g_jn_begin_ms;
	g_jn_prog_ms_r = g_jn_begin_ms;
	motor_set(left, right);
}

/* Rescan reverse that follows the line (Jin, 19 Nov). Before, the
   reverse only held both wheels even and ignored the line, so it could
   end beside the line. Now: the same wheel-difference hold as before
   (it keeps the heading: steering on sensors that trail behind the
   wheels, as they do when reversing, wobbles more and more on its own),
   plus a gentle turn toward the line when only one front sensor is on
   black, the same direction as forward line following (line under the
   left sensor -> turn left). Both sensors white for JN_REV_LOST_MS:
   the line is lost, stop instead of reversing off the track. */
#define JN_REV_STEER_PWM	10	/* TUNE: line correction while reversing */
#define JN_REV_LOST_MS		600U	/* both white this long = line lost */
LOCAL UW	g_jn_rev_white_ms;	/* when both sensors went white, 0 = not */

LOCAL BOOL jn_reverse_follow(W prog_l, W prog_r)
{
	W	hold = 0;
	W	steer = 0;
	W	mag_l;
	W	mag_r;
	UW const t = now_ms();

#if ENC_RIGHT_ENABLED
	hold = (prog_l - prog_r) / JN_HOLD_COUNTS_PER_PWM;
	if (hold > JN_HOLD_MAX_PWM)  { hold = JN_HOLD_MAX_PWM; }
	if (hold < -JN_HOLD_MAX_PWM) { hold = -JN_HOLD_MAX_PWM; }
#else
	(void)prog_l;
	(void)prog_r;
#endif
	if (g_lb && !g_rb) {
		steer = JN_REV_STEER_PWM;	/* line under the left sensor: turn left */
	} else if (g_rb && !g_lb) {
		steer = -JN_REV_STEER_PWM;
	} else {
		steer = 0;			/* centred, crossing, or in the gap */
	}
#if STEER_INVERT
	steer = -steer;
#endif
	if (!g_lb && !g_rb) {
		if (g_jn_rev_white_ms == 0U) {
			g_jn_rev_white_ms = t | 1U;
		} else if ((t - g_jn_rev_white_ms) >= JN_REV_LOST_MS) {
			return FALSE;		/* line lost */
		} else {
			/* briefly white: carry on */
		}
	} else {
		g_jn_rev_white_ms = 0U;
	}
	/* Reversing, both wheels run backwards. Turning left (CCW) while
	   reversing = the right wheel goes back slower than the left. The
	   hold slows whichever wheel is ahead, as in jn_hold_straight(). */
	mag_l = (W)g_jn_pwm - hold + steer;
	mag_r = (W)g_jn_pwm + hold - steer;
	mag_l = (mag_l < 0) ? 0 : mag_l;
	mag_r = (mag_r < 0) ? 0 : mag_r;
	motor_set((H)-mag_l, (H)-mag_r);
	return TRUE;
}

LOCAL void jn_pause(jn_phase_t phase)
{
	motor_set(0, 0);
	g_jn_phase    = phase;
	g_jn_until_ms = now_ms() + JN_PAUSE_MS;
}

/* Straight moves: slow the wheel that is ahead (same rule as the
   obstacle moves). */
LOCAL void jn_hold_straight(W prog_l, W prog_r)
{
#if ENC_RIGHT_ENABLED
	W hold = (prog_l - prog_r) / JN_HOLD_COUNTS_PER_PWM;
	W mag_l, mag_r;

	if (hold > JN_HOLD_MAX_PWM) {
		hold = JN_HOLD_MAX_PWM;
	} else if (hold < -JN_HOLD_MAX_PWM) {
		hold = -JN_HOLD_MAX_PWM;
	} else {
		/* in range */
	}
	if (hold == g_jn_hold) {
		return;
	}
	g_jn_hold = hold;
	mag_l = (W)g_jn_pwm - hold;
	mag_r = (W)g_jn_pwm + hold;
	mag_l = (mag_l < 0) ? 0 : mag_l;
	mag_r = (mag_r < 0) ? 0 : mag_r;
	if (g_jn_fwd_l) {
		motor_set((H)mag_l, (H)mag_r);
	} else {
		motor_set((H)-mag_l, (H)-mag_r);
	}
#else
	(void)prog_l;
	(void)prog_r;
#endif
}

/* Back to line following after a move; keeps the rescan count. */
LOCAL void jn_resume(void)
{
	g_jn_phase = JN_NONE;
	s3_request_line_reset();
	g_hold_until_ms = 0U;
	g_mode = CAR_FOLLOW;
	s1_post_state(ROBOT_LINE_FOLLOW);
}

/* Barcode move: stop, then creep onto the crossing (and spin). */
LOCAL void jn_begin_move(UB move)
{
	g_mode      = CAR_MANEUVER;
	g_jn_move   = move;
	g_jn_rescan = FALSE;
	g_jn_rescans = 0U;
	jn_pause(JN_PAUSE_IN);
	s1_post_state(ROBOT_TURNING);
}

/* Rescan: stop, reverse past the barcode, then line-follow back over it. */
LOCAL void jn_begin_rescan(W reverse_um)
{
	g_mode         = CAR_MANEUVER;
	g_jn_rescan    = TRUE;
	g_jn_rescan_um = reverse_um;
	jn_pause(JN_PAUSE_IN);
	s1_post_state(ROBOT_LINE_RECOVERY);
}

LOCAL void jn_fail(const char *why)
{
	W l, r;

	drv_encoder_read(&l, &r);
	tm_printf((UB *)"[jn] move FAILED: %s (L=%dmm R=%dmm of %dmm, %ums) - stopped\n", why,
		  (INT)(s2_counts_to_um(l - g_jn_start_l) / 1000),
		  (INT)(s2_counts_to_um(r - g_jn_start_r) / 1000),
		  (INT)(g_jn_target_um / 1000), (UINT)(now_ms() - g_jn_begin_ms));
	car_stop(ROBOT_ERROR);
}

/* What to do after the creep: spin, or (FORWARD) carry on. */
LOCAL void jn_after_creep(void)
{
	switch (g_jn_move) {
	case BARCMD_LEFT:
		jn_move(JN_TURN, (H)-JN_TURN_PWM, JN_TURN_PWM, JN_ARC_UM(90));
		break;
	case BARCMD_RIGHT:
		jn_move(JN_TURN, JN_TURN_PWM, (H)-JN_TURN_PWM, JN_ARC_UM(90));
		break;
	case BARCMD_U_TURN:
		jn_move(JN_TURN, JN_TURN_PWM, (H)-JN_TURN_PWM, JN_ARC_UM(180));
		break;
	default:	/* FORWARD */
		jn_pause(JN_PAUSE_OUT);
		break;
	}
}

/* Advance the junction move. Called every pass while CAR_MANEUVER. */
LOCAL void maneuver_step(void)
{
	UW t = now_ms();
	W  cl, cr, pl, pr, done;

	switch (g_jn_phase) {
	case JN_PAUSE_IN:
		if ((W)(t - g_jn_until_ms) >= 0) {
			if (g_jn_rescan) {
				g_jn_rev_white_ms = 0U;	/* Jin, 19 Nov */
				jn_move(JN_REVERSE, (H)-JN_DRIVE_PWM, (H)-JN_DRIVE_PWM,
					g_jn_rescan_um);
			} else {
				jn_move(JN_CREEP, JN_DRIVE_PWM, JN_DRIVE_PWM,
					(W)JN_CREEP_MM * 1000);
			}
		}
		break;

	case JN_CREEP:
	case JN_TURN:
	case JN_REVERSE:
		drv_encoder_read(&cl, &cr);
		pl = jn_progress(cl, g_jn_start_l, g_jn_fwd_l);
		pr = jn_progress(cr, g_jn_start_r, g_jn_fwd_r);
#if ENC_RIGHT_ENABLED
		done = (s2_counts_to_um(pl) + s2_counts_to_um(pr)) / 2;
#else
		done = s2_counts_to_um(pl);
#endif
		if (done >= g_jn_target_um) {
			tm_printf((UB *)"[jn] %s done: L=%dmm R=%dmm (target %dmm) %ums\n",
				  (g_jn_phase == JN_CREEP) ? "creep" :
				  (g_jn_phase == JN_TURN) ? "spin" : "reverse",
				  (INT)(s2_counts_to_um(pl) / 1000), (INT)(s2_counts_to_um(pr) / 1000),
				  (INT)(g_jn_target_um / 1000), (UINT)(t - g_jn_begin_ms));
			if (g_jn_phase == JN_CREEP) {
				motor_set(0, 0);
				jn_after_creep();
			} else {
				jn_pause(JN_PAUSE_OUT);
			}
		} else if (jn_stalled(pl, &g_jn_best_l, &g_jn_prog_ms_l, t)) {
			jn_fail("left wheel stalled");
#if ENC_RIGHT_ENABLED
		} else if (jn_stalled(pr, &g_jn_best_r, &g_jn_prog_ms_r, t)) {
			jn_fail("right wheel stalled");
#endif
		} else if ((t - g_jn_begin_ms) >= JN_MOVE_CAP_MS) {
			jn_fail("move took too long");
		} else if (g_jn_phase == JN_REVERSE) {
			if (!jn_reverse_follow(pl, pr)) {	/* Jin, 19 Nov */
				jn_fail("line lost while reversing");
			}
		} else if (g_jn_straight) {
			jn_hold_straight(pl, pr);
		} else {
			/* spinning */
		}
		break;

	case JN_PAUSE_OUT:
		if ((W)(t - g_jn_until_ms) >= 0) {
			jn_resume();
		}
		break;

	default:
		jn_resume();	/* no move set up: never sit in MANEUVER */
		break;
	}
}

/* Rising edge of the junction flag while following: decide. */
LOCAL void on_junction(void)
{
	UB	let = 0U;
	UB	mv  = 0U;
	W	rev_um;

	if (nav_has_pending(&g_nav, &let)) {
		BOOL mapped = nav_map_lookup(nav_map_snapshot(&g_nav_map), let, &mv)
			      ? TRUE : FALSE;

		nav_reset(&g_nav);		/* each barcode executes once */
		g_bar_unreadable = FALSE;
		s1_post_junction((char)('A' + let),
				 mapped ? to_telem_move(mv) : TELEM_MOVE_UNMAPPED);
		if (!mapped) {
			car_stop(ROBOT_STOPPED);
			g_jn_rescans = 0U;
			tm_printf((UB *)"[jn] junction: '%c' is not in the map - stopped\n",
				  'A' + let);
		} else if (JUNCTION_STOPS) {
			car_stop(ROBOT_STOPPED);
			tm_printf((UB *)"[jn] junction: '%c' -> %s (JUNCTION_STOPS=1: stopped)\n",
				  'A' + let, k_jn_move[mv & 3U]);
		} else {
			tm_printf((UB *)"[jn] junction: '%c' -> %s\n", 'A' + let,
				  k_jn_move[mv & 3U]);
			jn_begin_move(mv);
		}
		return;
	}

	/* Only a RECENT unreadable barcode is rescanned (Jin, 19 Nov): the
	   reverse is the distance travelled since that barcode's first bar
	   (wheel encoders) + JN_RESCAN_MARGIN_MM run-up. If that is more than
	   JN_RESCAN_MAX_MM, the "unreadable" was not this junction's barcode
	   (car test 19 Nov 19:55: it came from handling the car before
	   START), so it is dropped: no barcode here, straight on. The old
	   fixed JN_RESCAN_DEFAULT_MM reverse is no longer used. */
	if (g_bar_unreadable) {
		rev_um = (odo_um() - g_bar_unreadable_min_um) + ((W)JN_RESCAN_MARGIN_MM * 1000);
		if ((rev_um <= ((W)JN_RESCAN_MARGIN_MM * 1000)) ||
		    (rev_um > ((W)JN_RESCAN_MAX_MM * 1000))) {
			tm_printf((UB *)"[jn] unreadable barcode was %dmm back - not this junction's, ignored\n",
				  (INT)((rev_um / 1000) - (W)JN_RESCAN_MARGIN_MM));
			g_bar_unreadable = FALSE;
		}
	}
	if (g_bar_unreadable && (g_jn_rescans < JN_RESCAN_MAX) && !JUNCTION_STOPS) {
		g_jn_rescans++;
		g_bar_unreadable = FALSE;
		/* While driving back over the barcode, its bars look like a
		   junction (both front sensors black): car test 19 Nov 19:55
		   crept over the barcode as "no barcode - straight on". Ignore
		   junctions until the car is back within JN_RESCAN_ARRIVE_MM of
		   this one (Jin, 19 Nov). */
		g_jn_ignore_below_um = odo_um() - ((W)JN_RESCAN_ARRIVE_MM * 1000);
		g_jn_ignoring        = TRUE;
		tm_printf((UB *)"[jn] junction: barcode unreadable - rescan %u of %u, reversing %dmm\n",
			  g_jn_rescans, (UINT)JN_RESCAN_MAX, (INT)(rev_um / 1000));
		jn_begin_rescan(rev_um);
		return;
	}

#if JUNCTION_STOPS
	car_stop(ROBOT_STOPPED);
	if (g_bar_unreadable) {
		tm_printf((UB *)"[jn] junction: barcode still unreadable after %u rescans - stopped\n",
			  g_jn_rescans);
	} else {
		tm_printf((UB *)"[jn] junction: no barcode read - stopped\n");
	}
	g_bar_unreadable = FALSE;
	g_jn_rescans = 0U;
#else
	/* No barcode read (none there, or still unreadable after the
	   rescans): go straight over the junction and keep following the
	   line, instead of stopping (Jin, 19 Nov, for the team).  Same as a
	   FORWARD barcode: creep onto the crossing, then line following. */
	if (g_bar_unreadable) {
		tm_printf((UB *)"[jn] junction: barcode still unreadable after %u rescans - straight on\n",
			  g_jn_rescans);
	} else {
		tm_printf((UB *)"[jn] junction: no barcode - straight on\n");
	}
	s1_post_junction('?', to_telem_move(BARCMD_STRAIGHT));
	g_bar_unreadable = FALSE;
	jn_begin_move(BARCMD_STRAIGHT);		/* also clears g_jn_rescans */
#endif
}

/* Track barcode passes from S3's queued-edge counter: how many edges, and
   where (odometer) the rearmost one was, so a rescan knows how far to
   reverse. Called every pass. */
LOCAL void bar_track_edges(UW t)
{
	UB	src;
	UW	isr, samp, pushed, raw;
	W	odo;

	s3_barcode_diag(&src, &isr, &samp, &pushed, &raw);
	if (pushed == g_bar_last_pushed) {
		return;
	}
	odo = odo_um();
	if (!g_bar_pass_seen || ((t - g_bar_last_edge_ms) > JN_BAR_GAP_MS)) {
		g_bar_pass_edges  = 0U;
		g_bar_pass_min_um = odo;
		g_bar_pass_seen   = TRUE;
	}
	g_bar_pass_edges  += pushed - g_bar_last_pushed;
	g_bar_last_pushed  = pushed;
	g_bar_last_edge_ms = t;
	if (odo < g_bar_pass_min_um) {
		g_bar_pass_min_um = odo;
	}
}

#if APP_WITH_S5
/* =====================================================================
 *  Obstacle avoidance: motion executor for subsystem 5 (IF-09, IF-10).
 *
 *  S5 owns the avoidance: it profiles the obstacle, plans the bypass,
 *  drives it and recovers the line (project brief, Buddy 5 stages 4-5).
 *  This task only owns the wheels, so it:
 *    OBSTACLE  -> stops at once, CAR_AVOID, MOTORS_STOPPED after settling
 *    motion_q  -> drives that one move, then MOVE_DONE (or LINE_FOUND)
 *    CLEAR     -> S5 is done: line following again
 *    FAULT     -> stops, idle until START
 *  If the car is IDLE, nothing moves and every request is answered at
 *  once, so S5 can be tested on the bench with the wheels still.
 *
 *  Moves now end on wheel-encoder distance, measured from a snapshot
 *  taken when the move starts: straight moves on the mean of both
 *  wheels, spins on the mean wheel arc. A stalled wheel or the move cap
 *  stops the car and aborts S5 (car_stop), never MOVE_DONE.
 *  Gentler than the barcode turns on purpose: lower PWM, and the car
 *  pauses AV_GAP_MS between moves so it never flips straight from a spin
 *  into a drive. Power is constant for the whole move (no ramp). Stops
 *  are instant.
 * ===================================================================== */
#define AV_DRIVE_PWM		50	/* TUNE: straight moves. The line-follow
					   base (36) barely moves from a standstill */
#define AV_TURN_PWM		80	/* TUNE: spins. Softer than TURN_PWM (130),
					   so its own calibration below            */
#define AV_GAP_MS		150U	/* still between two moves                 */
#define AV_SETTLE_MS		150U	/* let the car stop rolling before S5 scans */
#define AV_IDLE_LIMIT_MS	15000U	/* CAR_AVOID, no move running and nothing
					   from S5 for this long: give up. A coarse
					   + fine scan takes up to ~4 s at 5 deg
					   steps (~8 s when 1 deg was tried); a
					   generous backstop for a silent S5.     */
#define AV_TRACK_MM		(110U)	/* Measured by Jin, 13 Nov: centre of the left
					   tyre to centre of the right tyre. Copy of
					   TRACK_MM in subsystem4/terrain_logic.c, which
					   S2 must not include. The effective pivot track
					   while spinning is assumed equal: TUNE if the
					   45 deg turns come out wrong.            */
#define AV_UM_PER_CM		(10000U)
#define AV_PI_360_E7		(87266U)	/* pi / 360 x 1e7 */
/* Arc per wheel in um for a spin of deg degrees: 45 -> 43196, 90 -> 86393.
   Largest case 255 * 110 * 87266 = 2.45e9, which fits in UW. */
#define AV_TURN_ARC_UM(deg)	(((UW)(deg) * AV_TRACK_MM * AV_PI_360_E7) / 10000U)
#define AV_STALL_MS		(500U)	/* TUNE (guess): a wheel making no progress this
					   long is stalled. Must cover the spin-up from
					   standstill.                              */
#define AV_STALL_STEP_COUNTS	(12)	/* Guess: about 1 mm (12 x 82 um). Ignores the
					   +-1 count jitter.                        */
#define AV_MOVE_CAP_MS		(14000U)	/* Backstop only: stall detection is the real
					   failure check. Must stay below S5's
					   OA_MOVE_TIMEOUT_MS (15000) minus AV_GAP_MS
					   and loop slack.                          */
/* Straight moves hold the car straight: the two motors are not equal, so
   at the same PWM one wheel runs ahead and the car drifts to one side.
   Every pass the wheel that is ahead (in encoder counts since the move
   began) is slowed and the other one sped up by 1 PWM per
   AV_HOLD_COUNTS_PER_PWM counts of difference, up to AV_HOLD_MAX_PWM.
   12 counts = 1 mm of wheel travel = 0.5 deg of heading on a 110 mm track.
   TUNE: lower COUNTS_PER_PWM = firmer hold (too low and it wobbles). */
#define AV_HOLD_COUNTS_PER_PWM	(12)
#define AV_HOLD_MAX_PWM		(15)

LOCAL BOOL		g_av_settling;	/* OBSTACLE: stopped, waiting to report */
LOCAL BOOL		g_av_moving;	/* a motion_q move is running */
LOCAL BOOL		g_av_pausing;	/* move over, AV_GAP_MS before replying */
LOCAL motor_mode_t	g_av_move;
LOCAL UB		g_av_param;	/* request param, for the print */
LOCAL W			g_av_target_um;	/* distance or arc per wheel */
LOCAL BOOL		g_av_fwd_l;	/* commanded direction per wheel */
LOCAL BOOL		g_av_fwd_r;
LOCAL BOOL		g_av_straight;		/* FWD/BWD: hold the two wheels level */
LOCAL H			g_av_pwm;		/* that move's PWM magnitude */
LOCAL W			g_av_hold;		/* steering correction last sent, PWM */
LOCAL BOOL		g_av_turn_to_line;	/* turn ends on the turning-side IR sensor */
LOCAL W			g_av_start_l;	/* counts at av_begin() */
LOCAL W			g_av_start_r;
LOCAL W			g_av_stop_l;	/* counts at av_finish() */
LOCAL W			g_av_stop_r;
LOCAL W			g_av_best_l;	/* best progress so far, counts */
LOCAL W			g_av_best_r;
LOCAL UW		g_av_prog_ms_l;	/* time of the last progress step */
LOCAL UW		g_av_prog_ms_r;
LOCAL UW		g_av_begin_ms;
LOCAL UW		g_av_stop_ms;
LOCAL UINT		g_av_reply;	/* reply to send after the pause */
LOCAL UW		g_av_until_ms;	/* end of the settle / pause */
LOCAL UW		g_av_heard_ms;	/* last message from S5 (watchdog) */
LOCAL UW		g_av_bad = 0U;	/* undecodable messages */

LOCAL const char * const k_obs_plan[] = { "STOP", "CONTINUE", "LEFT", "RIGHT",
					  "REVERSE", "?", "?", "?" };
LOCAL const char * const k_av_mode[] = { "STOP", "STEER", "FWD", "BWD",
					 "TURN_L", "TURN_R", "U_TURN", "FWD_TO_LINE" };

LOCAL void av_reply(UINT bits)
{
	(void)tk_set_flg(flg_avoid, bits);
}

LOCAL BOOL av_time_up(void)
{
	return ((W)(now_ms() - g_av_until_ms) >= 0) ? TRUE : FALSE;
}

/* Why S5 faulted: S5 sets its fault bit BEFORE it sends FAULT. */
LOCAL const char *s5_fault_cause(void)
{
	T_RFLG		rflg;
	const char	*p_cause = "unknown";

	if (tk_ref_flg(flg_faults, &rflg) == E_OK) {
		if ((rflg.flgptn & FAULT_BIT_US_NO_ECHO) != 0U) {
			p_cause = "ultrasonic no echo / stuck / too short";
		} else if ((rflg.flgptn & FAULT_BIT_NO_SAFE_RTE) != 0U) {
			p_cause = "no safe route (both sides blocked)";
		} else if ((rflg.flgptn & FAULT_BIT_NO_LINE) != 0U) {
			p_cause = "line not found after the bypass";
		} else if ((rflg.flgptn & FAULT_BIT_US_TIMEOUT) != 0U) {
			p_cause = "S2<->S5 handshake timeout";
		} else {
			/* keep "unknown" */
		}
	}
	return p_cause;
}

/* Instant stop (OBSTACLE, CLEAR): drop any pending reply too. */
/* Line search confirm (Jin, 19 Nov): passes in a row with a front sensor
   black before FWD_TO_LINE / a turn-to-line counts the line (~10 ms). */
#define AV_LINE_CONFIRM		(2U)
LOCAL UW		g_av_line_n;

LOCAL void av_halt(void)
{
	motor_set(0, 0);
	g_av_moving  = FALSE;
	g_av_pausing = FALSE;
}

/* End of a move: stop, stay still AV_GAP_MS, then reply. */
LOCAL void av_finish(UINT reply)
{
	drv_encoder_read(&g_av_stop_l, &g_av_stop_r);
	g_av_stop_ms  = now_ms();
	motor_set(0, 0);
	g_av_moving   = FALSE;
	g_av_pausing  = TRUE;
	g_av_reply    = reply;
	g_av_until_ms = now_ms() + AV_GAP_MS;
}

/* Start one move: constant PWM until the wheels have covered target_um
   (distance, or arc per wheel for a spin). The encoder counts at this
   moment are the zero of the move. */
LOCAL void av_begin(H left, H right, W target_um)
{
	drv_encoder_read(&g_av_start_l, &g_av_start_r);
	g_av_fwd_l     = (left >= 0) ? TRUE : FALSE;
	g_av_fwd_r     = (right >= 0) ? TRUE : FALSE;
	g_av_target_um = target_um;
	g_av_begin_ms  = now_ms();
	g_av_best_l    = 0;
	g_av_best_r    = 0;
	g_av_prog_ms_l = g_av_begin_ms;
	g_av_prog_ms_r = g_av_begin_ms;
	g_av_moving    = TRUE;
	g_av_pausing   = FALSE;
	g_av_line_n    = 0U;		/* Jin, 19 Nov: see avoid_step() */
	g_av_straight  = (left == right) ? TRUE : FALSE;	/* FWD / BWD */
	g_av_pwm       = (left < 0) ? (H)-left : left;
	g_av_hold      = 0;
	if (target_um <= 0) {
		av_finish(AVOID_EVT_MOVE_DONE);	/* nothing to drive */
		return;
	}
	motor_set(left, right);
}

/* Straight moves only: slow the wheel that is ahead, speed up the other.
   prog_l / prog_r: counts each wheel has moved in its commanded direction.
   Sends to Motor Control only when the correction changes, so mbf_motor
   is not flooded every 5 ms pass. */
LOCAL void av_hold_straight(W prog_l, W prog_r)
{
#if ENC_RIGHT_ENABLED
	W	hold = (prog_l - prog_r) / AV_HOLD_COUNTS_PER_PWM;
	W	mag_l;
	W	mag_r;

	if (hold > AV_HOLD_MAX_PWM) {
		hold = AV_HOLD_MAX_PWM;
	} else if (hold < -AV_HOLD_MAX_PWM) {
		hold = -AV_HOLD_MAX_PWM;
	} else {
		/* In range. */
	}
	if (hold == g_av_hold) {
		return;
	}
	g_av_hold = hold;

	mag_l = (W)g_av_pwm - hold;	/* left ahead -> hold > 0 -> left slower */
	mag_r = (W)g_av_pwm + hold;
	mag_l = (mag_l < 0) ? 0 : mag_l;
	mag_r = (mag_r < 0) ? 0 : mag_r;

	if (g_av_fwd_l) {
		motor_set((H)mag_l, (H)mag_r);
	} else {
		motor_set((H)-mag_l, (H)-mag_r);
	}
#else
	(void)prog_l;		/* right encoder off (UART build): nothing to compare */
	(void)prog_r;
#endif
}

/* Counts a wheel has moved in its commanded direction since av_begin(). */
LOCAL W av_progress(W now, W start, BOOL fwd)
{
	W d = fwd ? (now - start) : (start - now);

	return (d < 0) ? 0 : d;
}

/* Distance done in um: mean of both wheels (left only without the right encoder). */
LOCAL W av_done_um(W prog_l, W prog_r)
{
#if ENC_RIGHT_ENABLED
	return (s2_counts_to_um(prog_l) + s2_counts_to_um(prog_r)) / 2;
#else
	(void)prog_r;
	return s2_counts_to_um(prog_l);
#endif
}

/* One wheel: no new AV_STALL_STEP_COUNTS of progress for AV_STALL_MS. */
LOCAL BOOL av_wheel_stalled(W prog, W *p_best, UW *p_prog_ms, UW t)
{
	if (prog >= (*p_best + AV_STALL_STEP_COUNTS)) {
		*p_best    = prog;
		*p_prog_ms = t;
		return FALSE;
	}
	return ((t - *p_prog_ms) >= AV_STALL_MS) ? TRUE : FALSE;
}

/* One diagnostic line per move: asked vs. measured per wheel (mm). */
LOCAL void av_report(W cur_l, W cur_r, BOOL b_failed, const char *outcome)
{
	W const		l_mm  = s2_counts_to_um(cur_l - g_av_start_l) / 1000;
	W const		r_mm  = s2_counts_to_um(cur_r - g_av_start_r) / 1000;
	W const		ask_mm = g_av_target_um / 1000;
	const char	*p_unit = ((g_av_move == MCMD_TURN_LEFT) || (g_av_move == MCMD_TURN_RIGHT))
				  ? "deg" : "cm";
	const char	*p_mode = k_av_mode[(UINT)g_av_move & 7U];

	if (b_failed) {
#if ENC_RIGHT_ENABLED
		tm_printf((UB *)"[obs] move %s %u%s ask=%dmm L=%dmm R=%dmm t=%ums FAIL %s - stopped, S5 aborted\n",
			  p_mode, (UINT)g_av_param, p_unit, (INT)ask_mm, (INT)l_mm, (INT)r_mm,
			  (UINT)(now_ms() - g_av_begin_ms), outcome);
#else
		tm_printf((UB *)"[obs] move %s %u%s ask=%dmm L=%dmm R=%dmm t=%ums FAIL %s - stopped, S5 aborted Renc=off\n",
			  p_mode, (UINT)g_av_param, p_unit, (INT)ask_mm, (INT)l_mm, (INT)r_mm,
			  (UINT)(now_ms() - g_av_begin_ms), outcome);
#endif
	} else {
		W const	stop_l_mm = s2_counts_to_um(g_av_stop_l - g_av_start_l) / 1000;
		W const	stop_r_mm = s2_counts_to_um(g_av_stop_r - g_av_start_r) / 1000;

#if ENC_RIGHT_ENABLED
		tm_printf((UB *)"[obs] move %s %u%s ask=%dmm L=%dmm R=%dmm stopL=%d stopR=%d t=%ums %s\n",
			  p_mode, (UINT)g_av_param, p_unit, (INT)ask_mm, (INT)l_mm, (INT)r_mm,
			  (INT)stop_l_mm, (INT)stop_r_mm, (UINT)(g_av_stop_ms - g_av_begin_ms),
			  outcome);
#else
		tm_printf((UB *)"[obs] move %s %u%s ask=%dmm L=%dmm R=%dmm stopL=%d stopR=%d t=%ums %s Renc=off\n",
			  p_mode, (UINT)g_av_param, p_unit, (INT)ask_mm, (INT)l_mm, (INT)r_mm,
			  (INT)stop_l_mm, (INT)stop_r_mm, (UINT)(g_av_stop_ms - g_av_begin_ms),
			  outcome);
#endif
	}
}

/* A move failed: stop, say why, and car_stop() sends ABORT to S5. */
LOCAL void av_fail(W cur_l, W cur_r, const char *why)
{
	av_halt();
	av_report(cur_l, cur_r, TRUE, why);
	car_stop(ROBOT_ERROR);
}

/* One obstacle_q message. */
LOCAL void on_obstacle(obstacle_msg_t const *p)
{
	g_av_heard_ms = now_ms();

	switch (p->type) {
	case OBS_MSG_OBSTACLE:
		if (g_mode == CAR_IDLE) {
			av_reply(AVOID_EVT_MOTORS_STOPPED);	/* already still */
			tm_printf((UB *)"[obs] obstacle at %u cm (car idle)\n", (UINT)p->closest_cm);
			break;
		}
		/* Following, turning at a junction, or already avoiding
		   (something new ahead mid-bypass): stop now, S5 takes over. */
		g_mode        = CAR_AVOID;
		av_halt();
		g_av_settling = TRUE;
		g_av_until_ms = now_ms() + AV_SETTLE_MS;
		s1_post_state(ROBOT_OBSTACLE_SCAN);
		tm_printf((UB *)"[obs] OBSTACLE at %u cm - stopping\n", (UINT)p->closest_cm);
		break;

	case OBS_MSG_PLAN:
		tm_printf((UB *)"[obs] plan %s: closest=%u width=%u L=%u R=%u side=%u\n",
			  k_obs_plan[p->plan & 7U], (UINT)p->closest_cm, (UINT)p->width_cm,
			  (UINT)p->left_clear_cm, (UINT)p->right_clear_cm, (UINT)p->sidestep_cm);
		/* to the laptop dashboard (Jin, 19 Nov) */
		s1_post_obstacle(p->closest_cm, p->width_cm, p->left_clear_cm,
				 p->right_clear_cm, (UB)p->plan);
		break;

	case OBS_MSG_CLEAR:
		if (g_mode == CAR_AVOID) {
			av_halt();
			g_av_settling = FALSE;
			car_resume_follow();
			tm_printf((UB *)"[obs] S5 done - line following\n");
		}
		break;

	case OBS_MSG_FAULT:
	default:
		if (g_mode != CAR_IDLE) {
			car_stop(ROBOT_ERROR);
		}
		tm_printf((UB *)"[obs] S5 FAULT: %s - stopped. START / GP20 clears it.\n",
			  s5_fault_cause());
		break;
	}
}

/* One motion_q request from S5. */
LOCAL void on_motion(motor_cmd_msg_t m)
{
	motor_mode_t const	mode  = motor_cmd_get_mode(m);
	UW const		param = (UW)(UB)motor_cmd_get_param(m);	/* 0..255 */
	BOOL const		b_turn = ((mode == MCMD_TURN_LEFT) || (mode == MCMD_TURN_RIGHT))
				       ? TRUE : FALSE;
	BOOL const		b_turn_to_line = (b_turn && ((param & MCMD_PARAM_TO_LINE) != 0U))
					       ? TRUE : FALSE;
	UW const		deg = b_turn_to_line ? (param & MCMD_PARAM_VALUE_MASK) : param;

	g_av_heard_ms = now_ms();

	if (g_mode != CAR_AVOID) {
		/* Not avoiding (idle bench test): move nothing, answer now. */
		av_reply(((mode == MCMD_FWD_TO_LINE) || b_turn_to_line)
			 ? AVOID_EVT_LINE_FOUND : AVOID_EVT_MOVE_DONE);
		return;
	}

	g_av_param        = (UB)(b_turn ? deg : param);
	g_av_move         = mode;
	g_av_turn_to_line = b_turn_to_line;
	switch (mode) {
	case MCMD_FWD_DIST:
	case MCMD_FWD_TO_LINE:
		av_begin(AV_DRIVE_PWM, AV_DRIVE_PWM, (W)(param * AV_UM_PER_CM));
		s1_post_state((mode == MCMD_FWD_TO_LINE) ? ROBOT_LINE_RECOVERY : ROBOT_AVOIDING);
		break;
	case MCMD_BWD_DIST:
		av_begin((H)-AV_DRIVE_PWM, (H)-AV_DRIVE_PWM, (W)(param * AV_UM_PER_CM));
		break;
	case MCMD_TURN_LEFT:		/* same wheel signs as maneuver_step() */
		av_begin((H)-AV_TURN_PWM, AV_TURN_PWM, (W)AV_TURN_ARC_UM(deg));
		s1_post_state(b_turn_to_line ? ROBOT_LINE_RECOVERY : ROBOT_AVOIDING);
		break;
	case MCMD_TURN_RIGHT:
		av_begin(AV_TURN_PWM, (H)-AV_TURN_PWM, (W)AV_TURN_ARC_UM(deg));
		s1_post_state(b_turn_to_line ? ROBOT_LINE_RECOVERY : ROBOT_AVOIDING);
		break;
	case MCMD_STOP:
	default:			/* LINE_STEER / U_TURN: not used by S5 */
		av_halt();
		av_reply(AVOID_EVT_MOVE_DONE);
		break;
	}
}

/* A running move that ends on the IR line sensors: FWD_TO_LINE on either
   sensor; a turn-to-line on the sensor on the turning side (the one that
   reaches the line first as the car swings round to run along it). */
LOCAL BOOL av_line_seen(void)
{
	if (g_av_move == MCMD_FWD_TO_LINE) {
		return (g_lb || g_rb) ? TRUE : FALSE;
	}
	if (g_av_turn_to_line) {
		if (g_av_move == MCMD_TURN_LEFT) {
			return g_lb ? TRUE : FALSE;
		}
		return g_rb ? TRUE : FALSE;
	}
	return FALSE;
}

/* Advance the running move / pause / settle. Called every pass. */
LOCAL void avoid_step(void)
{
	UW	t;
	W	cl;
	W	cr;
	W	pl;
	W	pr;

	if (g_mode != CAR_AVOID) {
		return;
	}
	if (g_av_settling) {
		if (av_time_up()) {
			g_av_settling = FALSE;
			g_av_heard_ms = now_ms();
			av_reply(AVOID_EVT_MOTORS_STOPPED);
		}
	} else if (g_av_moving) {
		t = now_ms();
		/* The line counts only after AV_LINE_CONFIRM passes in a row
		   with black (Jin, 19 Nov; same as his 17 Nov fix): one speck
		   or a single noisy reading used to end the search early. */
		if (av_line_seen()) {
			g_av_line_n++;
		} else {
			g_av_line_n = 0U;
		}
		if (g_av_line_n >= AV_LINE_CONFIRM) {
			av_finish(AVOID_EVT_LINE_FOUND);
		} else {
			drv_encoder_read(&cl, &cr);
			pl = av_progress(cl, g_av_start_l, g_av_fwd_l);
			pr = av_progress(cr, g_av_start_r, g_av_fwd_r);
			if (av_done_um(pl, pr) >= g_av_target_um) {
				av_finish(AVOID_EVT_MOVE_DONE);
			} else if (av_wheel_stalled(pl, &g_av_best_l, &g_av_prog_ms_l, t)) {
				av_fail(cl, cr, "left stalled");
#if ENC_RIGHT_ENABLED
			} else if (av_wheel_stalled(pr, &g_av_best_r, &g_av_prog_ms_r, t)) {
				av_fail(cl, cr, "right stalled");
#endif
			} else if ((t - g_av_begin_ms) >= AV_MOVE_CAP_MS) {
				av_fail(cl, cr, "move too long");
			} else if (g_av_straight) {
				av_hold_straight(pl, pr);	/* still moving: keep it straight */
			} else {
				/* still moving (spin) */
			}
		}
	} else if (g_av_pausing) {
		if (av_time_up()) {
			drv_encoder_read(&cl, &cr);
			g_av_pausing  = FALSE;
			g_av_heard_ms = now_ms();
			av_reply(g_av_reply);
			av_report(cl, cr, FALSE, (g_av_reply == AVOID_EVT_LINE_FOUND) ? "LINE" : "DONE");
		}
	} else if ((now_ms() - g_av_heard_ms) > AV_IDLE_LIMIT_MS) {
		car_stop(ROBOT_ERROR);		/* sends ABORT to S5 */
		tm_printf((UB *)"[obs] no word from S5 for %u ms - stopped\n",
			  (UINT)AV_IDLE_LIMIT_MS);
	} else {
		/* waiting for S5's next move */
	}
}

/* Drain obstacle_q and motion_q. Never blocks. */
LOCAL void poll_obstacles(void)
{
	UB		buf[OBSTACLE_MSG_SIZE];
	obstacle_msg_t	msg;
	motor_cmd_msg_t	mv;

	while (tk_rcv_mbf(mbf_obstacle, buf, TMO_POL) == (INT)OBSTACLE_MSG_SIZE) {
		if (obstacle_msg_unpack(buf, &msg)) {
			on_obstacle(&msg);
		} else {
			g_av_bad++;
		}
	}
	while (tk_rcv_mbf(mbf_motion, &mv, TMO_POL) == (INT)sizeof(mv)) {
		on_motion(mv);
	}
}
#endif /* APP_WITH_S5 */

LOCAL telem_move_t to_telem_move(UB barcmd)
{
	return (barcmd < BARCMD_COUNT) ? (telem_move_t)barcmd : TELEM_MOVE_UNMAPPED;
}

/* ---- commands from Telemetry (MQTT), cmd_q ---- */
LOCAL void on_command(comms_cmd_t cmd)
{
	switch (cmd.type) {
	case CMD_START:
		if (g_mode == CAR_IDLE) {
			car_start();
		}
		break;
	case CMD_STOP:
		car_stop(ROBOT_STOPPED);
		tm_printf((UB *)"[car] STOP (MQTT)\n");
		break;
	case CMD_ESTOP:
		car_stop(ROBOT_STOPPED);
		tm_printf((UB *)"[car] ESTOP (MQTT)\n");
		break;
	case CMD_SET_SPEED:
		/* No encoders: "speed N" (cm/s, capped at 20 by comms) is scaled
		   to PWM, 20 -> PWM_AT_SPEED_MAX, keeping the M1:M2 ratio. */
		if ((cmd.arg > 0U) && (cmd.arg <= 20U)) {
			g_base_m1 = (H)(((INT)cmd.arg * PWM_AT_SPEED_MAX) / 20);
			g_base_m2 = (H)(((INT)g_base_m1 * BASE_PWM_M2) / BASE_PWM_M1);
			if (g_base_m2 > S2_PWM_MAX) {
				g_base_m2 = S2_PWM_MAX;
			}
			tm_printf((UB *)"[car] base PWM M1=%d M2=%d\n", g_base_m1, g_base_m2);
		}
		break;
	default:
		break;
	}
}

/* ---- GP20 button: press = start (idle) / stop (moving) ---- */
LOCAL void button_init(void)
{
	(void)gpio_set_pin(S2_PIN_BUTTON, GPIO_MODE_IN);
	out_w(S2_PAD(S2_PIN_BUTTON) + REG_ALIAS_CLR, S2_PAD_PDE);
	out_w(S2_PAD(S2_PIN_BUTTON) + REG_ALIAS_SET, S2_PAD_PUE);
}

LOCAL void button_poll(UW t)
{
	static UINT	stable = 1U;	/* released */
	static UINT	last_raw = 1U;
	static UW	changed_ms = 0U;
	UINT		raw = gpio_get_val(S2_PIN_BUTTON);

	if (raw != last_raw) {
		last_raw = raw;
		changed_ms = t;
		return;
	}
	if ((raw != stable) && ((t - changed_ms) >= S2_BUTTON_STABLE_MS)) {
		stable = raw;
		if (stable == 0U) {		/* pressed */
			if (g_mode == CAR_IDLE) {
				car_start();
			} else {
				car_stop(ROBOT_STOPPED);
				tm_printf((UB *)"[car] STOP (GP20 button)\n");
			}
		}
	}
}

/* ---- barcode_q: VALID -> pending letter for the next junction ---- */
LOCAL void on_barcode(barcode_t res)
{
	if (barcode_is_valid(res)) {
		UB mv = 0U;
		UB mapped = nav_map_lookup(nav_map_snapshot(&g_nav_map),
					   barcode_get_letter(res), &mv);
		tm_printf((UB *)"[bar] VALID '%c'%s\n", barcode_get_char(res),
			  mapped ? "" : " (not in map)");
		nav_on_barcode(&g_nav, res);
		g_bar_unreadable = FALSE;	/* a good read wins */
		s1_post_barcode(barcode_get_char(res),
				mapped ? to_telem_move(mv) : TELEM_MOVE_UNMAPPED);
	} else {
		/* A real attempt (enough edges in this pass), not flicker:
		   the next junction may rescan it. */
		if (g_bar_pass_seen && (g_bar_pass_edges >= JN_BAR_MIN_EDGES)) {
			g_bar_unreadable        = TRUE;
			g_bar_unreadable_min_um = g_bar_pass_min_um;
		}
		s1_post_barcode('?', TELEM_MOVE_UNMAPPED);	/* printed by S3 */
	}
}

/* ---- one line_err_q message: junction trigger + steering ---- */
LOCAL void on_line(line_err_t le)
{
	static UW	n_line = 0U;
	INT		err   = line_err_get_error(le);
	UB		l_blk = line_err_get_left_black(le);
	UB		r_blk = line_err_get_right_black(le);

	g_lb = l_blk;
	g_rb = r_blk;

	if ((++n_line % S2_TELEM_LINE_EVERY) == 0U) {
		s1_post_line((UB)((l_blk ? 0x1U : 0U) | (r_blk ? 0x2U : 0U)),
			     (H)(err / 8), line_err_get_lost(le) ? FALSE : TRUE);
	}

	/* junction: rising edge of "both black", acted on only while following */
	{
		UB jn = line_err_get_junction(le);
		UB rising = (jn && !g_jn_prev) ? 1U : 0U;

		g_jn_prev = jn;
		/* Rescan: skip the barcode's bars on the way back (Jin, 19 Nov). */
		if (g_jn_ignoring && (odo_um() >= g_jn_ignore_below_um)) {
			g_jn_ignoring = FALSE;
		}
		if (rising && (g_mode == CAR_FOLLOW) && !line_err_get_lost(le)) {
			if (g_jn_ignoring) {
				tm_printf((UB *)"[jn] both black %dmm before the junction - barcode bar, ignored\n",
					  (INT)((g_jn_ignore_below_um - odo_um()) / 1000) +
					  (INT)JN_RESCAN_ARRIVE_MM);
			} else {
				on_junction();
			}
		}
	}

	if (g_mode == CAR_FOLLOW) {
#if STEERING_ENABLED
		if (line_err_get_lost(le)) {
			car_stop(ROBOT_LINE_RECOVERY);
			tm_printf((UB *)"[car] line data unusable - stopped\n");
		} else {
			/* Proportional steer.  err is +-511: 0 = line centred
			   between the sensors, + = line under the LEFT sensor.
			   A small offset gets a small correction; only a large one
			   saturates, which is what holds a bend.  When both are
			   white the line is in the gap: keep a light steer for
			   STEER_HOLD_MS so a curve is carried. */
			H steer;
			if (l_blk && !r_blk) {
				steer = STEER_MAX;
				g_hold_until_ms = now_ms() + STEER_HOLD_MS;
			} else if (r_blk && !l_blk) {
				steer = -STEER_MAX;
				g_hold_until_ms = now_ms() + STEER_HOLD_MS;
			} else if (!l_blk && !r_blk) {
				if ((W)(now_ms() - g_hold_until_ms) < 0) {
					steer = (line_err_get_last_side(le) == LINE_SIDE_LEFT)
					      ? STEER_HOLD_LEVEL : -STEER_HOLD_LEVEL;
				} else {
					steer = (H)(err / STEER_ERR_DIV);
				}
			} else {
				steer = 0;	/* both black: crossing, no steering */
			}
#if STEER_INVERT
			steer = (H)-steer;
#endif
			if (steer >  STEER_MAX) { steer =  STEER_MAX; }
			if (steer < -STEER_MAX) { steer = -STEER_MAX; }
			/* steer > 0 -> left wheel slower, right faster */
			g_out_l = (H)(BASE_LEFT  - steer);
			g_out_r = (H)(BASE_RIGHT + steer);
			/* never reverse a wheel by steering: coast instead */
			if (g_out_l < 0) { g_out_l = 0; }
			if (g_out_r < 0) { g_out_r = 0; }
			motor_set(g_out_l, g_out_r);
		}
#else
		g_out_l = BASE_LEFT;
		g_out_r = BASE_RIGHT;
		motor_set(g_out_l, g_out_r);
#endif
	} else if (g_mode == CAR_MANEUVER) {
		maneuver_step();
	} else {
		/* idle */
	}
}

/* Lifted / overturned fault, cleared once the car is down again (Jin,
   19 Nov). S4 sets fault bit 2 when the car is picked up or tipped over;
   only Mission Control clears fault bits, and until now only START did,
   so the dashboard kept showing "lifted or overturned" after the car was
   simply carried back to the start. Now, while the car is stopped
   (CAR_IDLE) and S4 reports it ON THE FLOOR again, the bit is cleared.
   While driving nothing changes: a lift still stops the car and stays
   shown until START. */
LOCAL void lift_fault_clear(void)
{
	T_RFLG	rflg;

	if (g_mode != CAR_IDLE) {
		return;
	}
	if ((tk_ref_flg(flg_faults, &rflg) != E_OK) ||
	    ((rflg.flgptn & FAULT_BIT_COLLISION) == 0U)) {
		return;
	}
	if (s4_report_lift(s4_report()) == 0U) {	/* TERRAIN_ON_FLOOR */
		(void)tk_clr_flg(flg_faults, ~(UINT)FAULT_BIT_COLLISION);
		tm_printf((UB *)"[car] back on the floor - lifted/overturned fault cleared\n");
	}
}

EXPORT void mission_task(INT stacd, void *exinf)
{
	line_err_t	le = line_err_pack(0, 0U, 1U);
	line_err_t	msg;
	barcode_t	b;
	comms_cmd_t	cmd;
	UW		next_print_ms;
	UW		t;
	INT		n;

	(void)stacd; (void)exinf;

	nav_reset(&g_nav);
	button_init();

	s3_wait_console();
	tk_dly_tsk(300);
	tm_printf((UB *)"\n=== robocar uT-Kernel v1.0 (port of integrated_car v0.8, comms v0.3.1)\n"
		  "    base M1=%d M2=%d, start = %s, steer_invert = %d, junction = %s ===\n",
		  BASE_PWM_M1, BASE_PWM_M2,
		  STEERING_ENABLED ? "line steering" : "straight, both wheels",
		  STEER_INVERT, JUNCTION_STOPS ? "stop" : "barcode turn");

	next_print_ms = now_ms();
#if APP_WITH_S5
	/* From here on obstacle_q is drained every pass: let S5 start pinging. */
	(void)tk_set_flg(flg_avoid, AVOID_EVT_MC_READY);
#endif
	for (;;) {
		n = tk_rcv_mbf(mbf_line_err, &msg, S2_LINE_WAIT_MS);
		t = now_ms();

		/* commands first (main.c: comms_poll() ran first each pass) */
		while (tk_rcv_mbf(mbf_cmd, &cmd, TMO_POL) == (INT)sizeof(cmd)) {
			on_command(cmd);
		}
		button_poll(t);
		lift_fault_clear();	/* Jin, 19 Nov */

#if APP_WITH_S5
		poll_obstacles();	/* before the line: a stop wins over steering */
#endif
		if (n == (INT)sizeof(msg)) {
			/* after START the line task resets its state; drop messages
			   made with the old state until it has done so (IF-03) */
			if (!s3_line_reset_pending()) {
				le = msg;
				on_line(le);
			}
		} else if (g_mode == CAR_MANEUVER) {
			maneuver_step();	/* no line data: keep the timing */
		} else {
			/* nothing this pass */
		}

		while (tk_rcv_mbf(mbf_barcode, &b, TMO_POL) == (INT)sizeof(b)) {
			on_barcode(b);
		}
		bar_track_edges(t);	/* after on_barcode: a result closes its pass first */
#if APP_WITH_S5
		avoid_step();		/* after on_line(): uses fresh g_lb / g_rb */
#endif

		/* status line, 2 Hz (none before READY, like integrated_car) */
		if (((W)(t - next_print_ms) >= 0) && s3_is_calibrated()) {
			UB pend = 0U;
			UB bsrc = 0U;
			UW bisr = 0U, bsamp = 0U, bpush = 0U, braw = 0U;
			next_print_ms += S2_STATUS_MS;
			if ((W)(t - next_print_ms) >= 0) {
				next_print_ms = t + S2_STATUS_MS;	/* after calibration */
			}
			s3_barcode_diag(&bsrc, &bisr, &bsamp, &bpush, &braw);
			tm_printf((UB *)"mode=%u L=%c R=%c err=%4d jn=%u lost=%u pending=%c pwm=%d/%d link=%s"
				  " bar=%c irq%u smp%u q%u raw%u\n",
				  (UINT)g_mode, g_lb ? 'B' : 'W', g_rb ? 'B' : 'W',
				  line_err_get_error(le), (UINT)line_err_get_junction(le),
				  (UINT)line_err_get_lost(le),
				  nav_has_pending(&g_nav, &pend) ? (char)('A' + pend) : '-',
				  (g_mode == CAR_FOLLOW) ? g_out_l : 0,
				  (g_mode == CAR_FOLLOW) ? g_out_r : 0,
				  s1_link_name(), (char)bsrc, (UINT)bisr, (UINT)bsamp, (UINT)bpush,
				  (UINT)braw);
		}
		(void)tk_set_flg(flg_hb, HB_MISSION_CONTROL);
	}
}

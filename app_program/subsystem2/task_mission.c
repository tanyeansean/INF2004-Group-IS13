/*
 *	task_mission.c
 *	Subsystem 2 -- Mission Control task.  Port of the car logic in
 *	integrated_car/main.c v0.8 (same rules, same numbers):
 *
 *	  both white          -> straight          PWM 36 / 36
 *	  both white, <= 80 ms since one sensor last saw the line
 *	                      -> light steer, +-16 (bend recovery)
 *	  one sensor black    -> steer up to +-36, scaled by err/14
 *	  both black >= 30 ms -> junction: STOP (JUNCTION_STOPS = 1), or the
 *	                         timed barcode turn (JUNCTION_STOPS = 0)
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
#if APP_WITH_S5
#include "../subsystem5/subsystem5.h"	/* flg_avoid, AVOID_EVT_* (IF-09/10 replies) */
#endif

/* ---- Tuning (open loop: no encoders yet - same as integrated_car) ---- */
#ifndef STEERING_ENABLED
#define STEERING_ENABLED	1
#endif
#define BASE_PWM_M1		36
#define BASE_PWM_M2		36	/* same as M1: no trim, both wheels equal */
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
#define JUNCTION_STOPS		1	/* 1 = stop at a junction, no barcode turn */
#endif
#ifndef PWM_AT_SPEED_MAX
#define PWM_AT_SPEED_MAX	85	/* PWM for "speed 20" (not yet measured) */
#endif
#define TURN_PWM		130
#define TURN_90_MS		450U
#define TURN_180_MS		900U
#define CROSS_MS		300U
#define MOTOR_LEFT_IS_M1	1

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
LOCAL UW		g_man_end_ms;
LOCAL H			g_man_left, g_man_right;
LOCAL UB		g_man_phase;
LOCAL UB		g_man_move;
LOCAL nav_state_t	g_nav;
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
#if APP_WITH_S5
	/* START also clears S5's latched fault: only Mission Control clears
	   fault bits. RESUME takes S5 out of its FAULT state. */
	(void)tk_clr_flg(flg_faults, ~(UINT)(FAULT_BIT_US_NO_ECHO | FAULT_BIT_NO_SAFE_RTE |
					      FAULT_BIT_US_TIMEOUT | FAULT_BIT_NO_LINE));
	if (s5_get_state() == S5_STATE_FAULT) {
		(void)tk_set_flg(flg_avoid, AVOID_EVT_RESUME);
	}
#endif
	car_resume_follow();
	tm_printf((UB *)"[car] START (base PWM M1=%d M2=%d)\n", g_base_m1, g_base_m2);
}

LOCAL void maneuver_begin(UB move)
{
	g_mode       = CAR_MANEUVER;
	g_man_move   = move;
	g_man_phase  = 0U;
	g_man_left   = BASE_LEFT;
	g_man_right  = BASE_RIGHT;
	g_man_end_ms = now_ms() + CROSS_MS;
	motor_set(g_man_left, g_man_right);
	s1_post_state(ROBOT_TURNING);
}

LOCAL void maneuver_step(void)
{
	if ((W)(now_ms() - g_man_end_ms) < 0) {
		return;
	}
	if ((g_man_phase == 0U) && (g_man_move != BARCMD_STRAIGHT)) {
		g_man_phase = 1U;
		switch (g_man_move) {
		case BARCMD_LEFT:
			g_man_left = (H)-TURN_PWM; g_man_right = TURN_PWM;
			g_man_end_ms = now_ms() + TURN_90_MS;
			break;
		case BARCMD_RIGHT:
			g_man_left = TURN_PWM; g_man_right = (H)-TURN_PWM;
			g_man_end_ms = now_ms() + TURN_90_MS;
			break;
		default: /* BARCMD_U_TURN */
			g_man_left = TURN_PWM; g_man_right = (H)-TURN_PWM;
			g_man_end_ms = now_ms() + TURN_180_MS;
			break;
		}
		motor_set(g_man_left, g_man_right);
		return;
	}
	/* done: back to line following */
	s3_request_line_reset();
	g_hold_until_ms = 0U;
	g_mode = CAR_FOLLOW;
	s1_post_state(ROBOT_LINE_FOLLOW);
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
 *  Open loop until the encoders + PID exist: a distance is a time at
 *  AV_DRIVE_PWM, an angle a time at AV_TURN_PWM. Both need measuring.
 *  Gentler than the barcode turns on purpose: lower PWM, and the car
 *  pauses AV_GAP_MS between moves so it never flips straight from a spin
 *  into a drive. Power is constant for the whole move, so the distance
 *  stays proportional to the time (no ramp: it only made the short
 *  open-loop moves fall short). Stops are instant.
 * ===================================================================== */
#define AV_DRIVE_PWM		50	/* TUNE: straight moves. The line-follow
					   base (36) barely moves from a standstill */
#define AV_TURN_PWM		80	/* TUNE: spins. Softer than TURN_PWM (130),
					   so its own calibration below            */
#define AV_MS_PER_CM		50U	/* TUNE: command FWD 30 cm, measure, scale */
#define AV_MS_PER_DEG		8U	/* TUNE: command a 90 deg turn, measure   */
#define AV_GAP_MS		150U	/* still between two moves                 */
#define AV_SETTLE_MS		150U	/* let the car stop rolling before S5 scans */
#define AV_IDLE_LIMIT_MS	8000U	/* CAR_AVOID, no move running and nothing
					   from S5 for this long: give up. A
					   coarse + fine scan with re-pings takes
					   up to ~4 s, so keep well above that.  */

LOCAL BOOL		g_av_settling;	/* OBSTACLE: stopped, waiting to report */
LOCAL BOOL		g_av_moving;	/* a motion_q move is running */
LOCAL BOOL		g_av_pausing;	/* move over, AV_GAP_MS before replying */
LOCAL motor_mode_t	g_av_move;
LOCAL UINT		g_av_reply;	/* reply to send after the pause */
LOCAL UW		g_av_until_ms;	/* end of the move / settle / pause */
LOCAL UW		g_av_heard_ms;	/* last message from S5 (watchdog) */
LOCAL UW		g_av_bad = 0U;	/* undecodable messages */

LOCAL const char * const k_obs_plan[] = { "STOP", "CONTINUE", "LEFT", "RIGHT",
					  "REVERSE", "?", "?", "?" };

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
LOCAL void av_halt(void)
{
	motor_set(0, 0);
	g_av_moving  = FALSE;
	g_av_pausing = FALSE;
}

/* Start one move: constant PWM for len_ms. */
LOCAL void av_begin(H left, H right, UW len_ms)
{
	g_av_moving  = TRUE;
	g_av_pausing = FALSE;
	g_av_until_ms = now_ms() + len_ms;
	motor_set(left, right);
}

/* End of a move: stop, stay still AV_GAP_MS, then reply. */
LOCAL void av_finish(UINT reply)
{
	motor_set(0, 0);
	g_av_moving   = FALSE;
	g_av_pausing  = TRUE;
	g_av_reply    = reply;
	g_av_until_ms = now_ms() + AV_GAP_MS;
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

	g_av_heard_ms = now_ms();

	if (g_mode != CAR_AVOID) {
		/* Not avoiding (idle bench test): move nothing, answer now. */
		av_reply((mode == MCMD_FWD_TO_LINE) ? AVOID_EVT_LINE_FOUND : AVOID_EVT_MOVE_DONE);
		return;
	}

	g_av_move = mode;
	switch (mode) {
	case MCMD_FWD_DIST:
	case MCMD_FWD_TO_LINE:
		av_begin(AV_DRIVE_PWM, AV_DRIVE_PWM, param * AV_MS_PER_CM);
		s1_post_state((mode == MCMD_FWD_TO_LINE) ? ROBOT_LINE_RECOVERY : ROBOT_AVOIDING);
		break;
	case MCMD_BWD_DIST:
		av_begin((H)-AV_DRIVE_PWM, (H)-AV_DRIVE_PWM, param * AV_MS_PER_CM);
		break;
	case MCMD_TURN_LEFT:		/* same wheel signs as maneuver_step() */
		av_begin((H)-AV_TURN_PWM, AV_TURN_PWM, param * AV_MS_PER_DEG);
		s1_post_state(ROBOT_AVOIDING);
		break;
	case MCMD_TURN_RIGHT:
		av_begin(AV_TURN_PWM, (H)-AV_TURN_PWM, param * AV_MS_PER_DEG);
		s1_post_state(ROBOT_AVOIDING);
		break;
	case MCMD_STOP:
	default:			/* LINE_STEER / U_TURN: not used by S5 */
		av_halt();
		av_reply(AVOID_EVT_MOVE_DONE);
		break;
	}
}

/* Advance the running move / pause / settle. Called every pass. */
LOCAL void avoid_step(void)
{
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
		if ((g_av_move == MCMD_FWD_TO_LINE) && (g_lb || g_rb)) {
			av_finish(AVOID_EVT_LINE_FOUND);
		} else if (av_time_up()) {
			av_finish(AVOID_EVT_MOVE_DONE);
		} else {
			/* still moving */
		}
	} else if (g_av_pausing) {
		if (av_time_up()) {
			g_av_pausing  = FALSE;
			g_av_heard_ms = now_ms();
			av_reply(g_av_reply);
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
		s1_post_barcode(barcode_get_char(res),
				mapped ? to_telem_move(mv) : TELEM_MOVE_UNMAPPED);
	} else {
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
	UB		mv = 0U, let = 0U;
	UB		ev;

	g_lb = l_blk;
	g_rb = r_blk;

	if ((++n_line % S2_TELEM_LINE_EVERY) == 0U) {
		s1_post_line((UB)((l_blk ? 0x1U : 0U) | (r_blk ? 0x2U : 0U)),
			     (H)(err / 8), line_err_get_lost(le) ? FALSE : TRUE);
	}

	/* junction trigger: feed EVERY line message */
	ev = nav_on_line(&g_nav, le, nav_map_snapshot(&g_nav_map), &mv, &let);
	if (ev == NAV_EVT_MOVE) {
		tm_printf((UB *)"[nav] junction: '%c' -> move %u\n", 'A' + let, (UINT)mv);
		s1_post_junction((char)('A' + let), to_telem_move(mv));
		if ((g_mode == CAR_FOLLOW) && !JUNCTION_STOPS) {
			maneuver_begin(mv);
		}
	} else if (ev == NAV_EVT_UNMAPPED) {
		tm_printf((UB *)"[nav] junction: '%c' has NO mapping - driving on\n", 'A' + let);
		s1_post_junction((char)('A' + let), TELEM_MOVE_UNMAPPED);
	} else {
		/* keep going */
	}

	if (g_mode == CAR_FOLLOW) {
#if STEERING_ENABLED
		if (line_err_get_lost(le)) {
			car_stop(ROBOT_LINE_RECOVERY);
			tm_printf((UB *)"[car] line data unusable - stopped\n");
		} else if (line_err_get_junction(le) && JUNCTION_STOPS) {
			car_stop(ROBOT_STOPPED);
			tm_printf((UB *)"[car] JUNCTION (both black) - stopped. Send start to go again.\n");
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

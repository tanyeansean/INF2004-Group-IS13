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
#include "../subsystem5/subsystem5.h"	/* IF-06 reply flag flg_avoid, AVOID_EVT_* */
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

/* ---- Obstacle avoidance (IF-06, with subsystem 5).  Open loop like the
 *      barcode turns: every distance is a time at the base PWM.  All TUNE
 *      values need measuring on the car. ---- */
#define OBS_SETTLE_MS		150U	/* let the car stop rolling before S5 scans */
#define OBS_WAIT_PLAN_MS	5000U	/* stopped this long with no plan -> give up */
#define OBS_MS_PER_CM		65U	/* TUNE: ms to drive 1 cm at the base PWM */
#define OBS_TURN_45_MS		(TURN_90_MS / 2U)	/* spin 45 deg, same calibration as turns */
#define OBS_PASS_CM		30U	/* TUNE: drive this far alongside the obstacle */
#define OBS_SEEK_FACTOR		2U	/* look for the line up to 2x the expected distance */
#define OBS_DEFAULT_SIDE_CM	20U	/* if S5 gives no sidestep (255) */
#define OBS_REVERSE_CM		10U	/* TUNE: back off this far, then S5 rescans */
#define OBS_BYPASS_LIMIT_MS	8000U	/* whole bypass; S5's OA_BYPASS_TIMEOUT_MS is longer */
#define OBS_ROOT2_X100		141U	/* sqrt(2): 45 deg legs are 1.41x the sidestep */
#define OBS_SIDESTEP_NONE	255U	/* obstacle_msg.h: sidestep 255 = n/a */

typedef enum {
	CAR_IDLE = 0, CAR_FOLLOW, CAR_MANEUVER,
	CAR_AVOID		/* stopped for / driving around an obstacle (S5) */
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
	/* Leaving an avoid without finishing it (give-up, STOP, ESTOP, GP20):
	   S5 is waiting for MOTORS_STOPPED or LINE_REACQUIRED that will never
	   come, and would latch a TIMEOUT "fault". Tell it to stand down. */
	if (g_mode == CAR_AVOID) {
		(void)tk_set_flg(flg_avoid, AVOID_EVT_ABORT);
	}
#endif
	g_mode = CAR_IDLE;
	g_hold_until_ms = 0U;
	s1_post_state(why);
}

/* Back to line following without the START checks (after a bypass). */
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
	/* A START also clears S5's latched fault: drop its fault bits (only
	   Mission Control clears fault bits) and tell it to resume. */
	(void)tk_clr_flg(flg_faults, ~(UINT)(FAULT_BIT_US_NO_ECHO | FAULT_BIT_NO_SAFE_RTE |
					     FAULT_BIT_US_TIMEOUT));
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
 *  Obstacle avoidance (IF-06).  Subsystem 5 senses and plans; this task
 *  owns the wheels and drives the bypass.  Handshake:
 *
 *    S5: OBSTACLE  -> stop at once, wait OBS_SETTLE_MS, reply MOTORS_STOPPED
 *    S5: PLAN LEFT/RIGHT -> timed bypass (below), reply LINE_REACQUIRED
 *    S5: PLAN REVERSE    -> back off OBS_REVERSE_CM, reply MOTORS_STOPPED
 *    S5: PLAN STOP / FAULT -> stop, stay stopped until START
 *    S5: CLEAR (while stopped for it) -> obstacle gone, resume line following
 *    S2: MC_READY once, just before the main loop -> S5 starts pinging
 *    S2: ABORT whenever the car leaves CAR_AVOID via car_stop() (give-up,
 *        STOP, ESTOP, GP20) -> S5 returns to MONITOR instead of timing out
 *
 *  Bypass on side D (D = left or right), all open loop:
 *    TURN_OUT  spin 45 deg toward D
 *    OUT       forward sidestep * sqrt(2)   (ends sidestep cm off the line)
 *    STRAIGHT  spin 45 deg back             (parallel to the line)
 *    PASS      forward OBS_PASS_CM          (alongside the obstacle)
 *    TURN_IN   spin 45 deg toward the line
 *    SEEK      forward until a front IR sensor sees black (or give up)
 *    ALIGN     spin 45 deg back toward D    (heading along the line again)
 *  then line following takes over.  S5 keeps pinging ahead the whole time
 *  and sends a new OBSTACLE if something gets too close.
 *  If the car is NOT driving (idle), S5's messages are answered at once and
 *  nothing moves, so S5 can be tested on the bench with the wheels still.
 * ===================================================================== */
typedef enum {
	AV_STOPPING = 0, AV_WAIT_PLAN, AV_REVERSE,
	AV_TURN_OUT, AV_OUT, AV_STRAIGHT, AV_PASS, AV_TURN_IN, AV_SEEK, AV_ALIGN
} avoid_phase_t;

LOCAL avoid_phase_t	g_av_phase;
LOCAL UW		g_av_until_ms;	/* end of the current timed step */
LOCAL UW		g_av_start_ms;	/* bypass start (OBS_BYPASS_LIMIT_MS) */
LOCAL UW		g_av_seek_from_ms;	/* SEEK: ignore black before this */
LOCAL UW		g_av_diag_ms;	/* time of one 45-degree leg */
LOCAL H			g_av_dir;	/* +1 = bypass on the LEFT, -1 = RIGHT */
LOCAL UW		g_obs_bad = 0U;	/* undecodable obstacle_q messages */

LOCAL const char * const k_obs_plan[] = { "STOP", "CONTINUE", "LEFT", "RIGHT",
					  "REVERSE", "?", "?", "?" };

/* Why S5 faulted: OBS_MSG_FAULT carries no cause, but S5 sets its fault
   bit in flg_faults BEFORE it sends the message. */
LOCAL const char *s5_fault_cause(void)
{
	T_RFLG		rflg;
	const char	*p_cause = "unknown (no S5 fault bit set)";

	if (tk_ref_flg(flg_faults, &rflg) != E_OK) {
		p_cause = "unknown (flg_faults unreadable)";
	} else if ((rflg.flgptn & FAULT_BIT_US_NO_ECHO) != 0U) {
		p_cause = "ultrasonic NO ECHO / ECHO stuck / echo too short (see [S5 dbg])";
	} else if ((rflg.flgptn & FAULT_BIT_US_TIMEOUT) != 0U) {
		p_cause = "S2<->S5 handshake TIMEOUT (not the sensor)";
	} else if ((rflg.flgptn & FAULT_BIT_NO_SAFE_RTE) != 0U) {
		p_cause = "NO SAFE ROUTE (both sides blocked)";
	} else {
		/* keep "unknown" */
	}
	return p_cause;
}

LOCAL void av_reply(UINT bits)
{
	(void)tk_set_flg(flg_avoid, bits);
}

LOCAL BOOL av_time_up(void)
{
	return ((W)(now_ms() - g_av_until_ms) >= 0) ? TRUE : FALSE;
}

LOCAL void av_drive(H left, H right, UW ms)
{
	motor_set(left, right);
	g_av_until_ms = now_ms() + ms;
}

/* Spin in place 45 deg (or for ms): toward = +1 left, -1 right.  Same
   wheel signs as the barcode turns in maneuver_step(). */
LOCAL void av_spin(H toward, UW ms)
{
	av_drive((H)(-toward * TURN_PWM), (H)(toward * TURN_PWM), ms);
}

LOCAL void av_forward(UW ms)
{
	av_drive(BASE_LEFT, BASE_RIGHT, ms);
}

/* Motors off now; MOTORS_STOPPED goes to S5 once the car has settled. */
LOCAL void av_stop_then_report(void)
{
	motor_set(0, 0);
	g_av_phase    = AV_STOPPING;
	g_av_until_ms = now_ms() + OBS_SETTLE_MS;
}

LOCAL void av_give_up(const char *why)
{
	car_stop(ROBOT_ERROR);
	tm_printf((UB *)"[obs] %s - stopped. Send start / press GP20 to go again.\n", why);
}

LOCAL void av_begin_bypass(H dir, UB sidestep_cm)
{
	UW const side = (sidestep_cm == OBS_SIDESTEP_NONE) ? OBS_DEFAULT_SIDE_CM
							      : (UW)sidestep_cm;

	g_av_dir      = dir;
	g_av_diag_ms  = (side * OBS_ROOT2_X100 * OBS_MS_PER_CM) / 100U;
	g_av_start_ms = now_ms();
	g_av_phase    = AV_TURN_OUT;
	av_spin(dir, OBS_TURN_45_MS);
	s1_post_state(ROBOT_AVOIDING);
	tm_printf((UB *)"[obs] bypass %s, sidestep %u cm (leg %u ms)\n",
		  (dir > 0) ? "LEFT" : "RIGHT", (UINT)side, (UINT)g_av_diag_ms);
}

/* One message from obstacle_q. */
LOCAL void on_obstacle(obstacle_msg_t const *p)
{
	BOOL const avoiding = (g_mode == CAR_AVOID) ? TRUE : FALSE;

	switch (p->type) {
	case OBS_MSG_OBSTACLE:
		if ((g_mode == CAR_FOLLOW) || (g_mode == CAR_MANEUVER) || avoiding) {
			g_mode = CAR_AVOID;		/* cancels a barcode turn */
			av_stop_then_report();
			s1_post_state(ROBOT_OBSTACLE_SCAN);
			tm_printf((UB *)"[obs] OBSTACLE at %u cm - stopping\n", (UINT)p->closest_cm);
		} else {
			av_reply(AVOID_EVT_MOTORS_STOPPED);	/* idle: already stopped */
			tm_printf((UB *)"[obs] obstacle at %u cm (car idle)\n", (UINT)p->closest_cm);
		}
		break;

	case OBS_MSG_PLAN:
		tm_printf((UB *)"[obs] plan %s: closest=%u width=%u L=%u R=%u side=%u\n",
			  k_obs_plan[p->plan & 7U], (UINT)p->closest_cm, (UINT)p->width_cm,
			  (UINT)p->left_clear_cm, (UINT)p->right_clear_cm, (UINT)p->sidestep_cm);
		if (!avoiding || (g_av_phase != AV_WAIT_PLAN)) {
			/* Not driving: move nothing, just let S5 carry on. */
			if ((p->plan == OBS_PLAN_LEFT) || (p->plan == OBS_PLAN_RIGHT)) {
				av_reply(AVOID_EVT_LINE_REACQUIRED);
			} else if (p->plan == OBS_PLAN_REVERSE) {
				av_reply(AVOID_EVT_MOTORS_STOPPED);
			} else {
				/* STOP: FAULT follows */
			}
			break;
		}
		switch (p->plan) {
		case OBS_PLAN_LEFT:
			av_begin_bypass(1, p->sidestep_cm);
			break;
		case OBS_PLAN_RIGHT:
			av_begin_bypass(-1, p->sidestep_cm);
			break;
		case OBS_PLAN_REVERSE:
			g_av_phase = AV_REVERSE;
			av_drive((H)-BASE_LEFT, (H)-BASE_RIGHT, OBS_REVERSE_CM * OBS_MS_PER_CM);
			tm_printf((UB *)"[obs] reversing %u cm, then S5 rescans\n", (UINT)OBS_REVERSE_CM);
			break;
		case OBS_PLAN_CONTINUE:
			car_resume_follow();
			break;
		case OBS_PLAN_STOP:
		default:
			av_give_up("no safe route around the obstacle");
			break;
		}
		break;

	case OBS_MSG_CLEAR:
		if (avoiding && (g_av_phase == AV_WAIT_PLAN)) {
			tm_printf((UB *)"[obs] path clear - resuming line following\n");
			car_resume_follow();
		}
		break;

	case OBS_MSG_FAULT:
	default:
		if (g_mode != CAR_IDLE) {
			av_give_up("obstacle subsystem (S5) FAULT");
		}
		tm_printf((UB *)"[obs] S5 FAULT, cause: %s. START / GP20 clears it.\n",
			  s5_fault_cause());
		break;
	}
}

/* Advance the timed steps.  Called every pass (line message or timeout). */
LOCAL void avoid_step(void)
{
	if (g_mode != CAR_AVOID) {
		return;
	}
	if ((g_av_phase >= AV_TURN_OUT) &&
	    ((now_ms() - g_av_start_ms) > OBS_BYPASS_LIMIT_MS)) {
		av_give_up("bypass took too long");
		return;
	}

	switch (g_av_phase) {
	case AV_STOPPING:
		if (av_time_up()) {
			av_reply(AVOID_EVT_MOTORS_STOPPED);
			g_av_phase    = AV_WAIT_PLAN;
			g_av_until_ms = now_ms() + OBS_WAIT_PLAN_MS;
		}
		break;
	case AV_WAIT_PLAN:
		if (av_time_up()) {
			av_give_up("no plan from the obstacle sensor");
		}
		break;
	case AV_REVERSE:
		if (av_time_up()) {
			av_stop_then_report();
		}
		break;
	case AV_TURN_OUT:
		if (av_time_up()) {
			g_av_phase = AV_OUT;
			av_forward(g_av_diag_ms);
		}
		break;
	case AV_OUT:
		if (av_time_up()) {
			g_av_phase = AV_STRAIGHT;
			av_spin((H)-g_av_dir, OBS_TURN_45_MS);
		}
		break;
	case AV_STRAIGHT:
		if (av_time_up()) {
			g_av_phase = AV_PASS;
			av_forward(OBS_PASS_CM * OBS_MS_PER_CM);
		}
		break;
	case AV_PASS:
		if (av_time_up()) {
			g_av_phase = AV_TURN_IN;
			av_spin((H)-g_av_dir, OBS_TURN_45_MS);
		}
		break;
	case AV_TURN_IN:
		if (av_time_up()) {
			g_av_phase        = AV_SEEK;
			g_av_seek_from_ms = now_ms() + (g_av_diag_ms / 2U);
			av_forward(g_av_diag_ms * OBS_SEEK_FACTOR);
			s1_post_state(ROBOT_LINE_RECOVERY);
		}
		break;
	case AV_SEEK:
		if ((g_lb || g_rb) && ((W)(now_ms() - g_av_seek_from_ms) >= 0)) {
			g_av_phase = AV_ALIGN;
			av_spin(g_av_dir, OBS_TURN_45_MS);
			tm_printf((UB *)"[obs] line found - aligning\n");
		} else if (av_time_up()) {
			av_give_up("line not found after the bypass");
		} else {
			/* keep looking */
		}
		break;
	case AV_ALIGN:
	default:
		if (av_time_up()) {
			av_reply(AVOID_EVT_LINE_REACQUIRED);
			tm_printf((UB *)"[obs] bypass done (%u ms) - line following\n",
				  (UINT)(now_ms() - g_av_start_ms));
			car_resume_follow();
		}
		break;
	}
}

/* Drain obstacle_q.  Never blocks. */
LOCAL void poll_obstacles(void)
{
	UB		buf[OBSTACLE_MSG_SIZE];
	obstacle_msg_t	msg;

	while (tk_rcv_mbf(mbf_obstacle, buf, TMO_POL) == (INT)OBSTACLE_MSG_SIZE) {
		if (obstacle_msg_unpack(buf, &msg)) {
			on_obstacle(&msg);
		} else {
			g_obs_bad++;
		}
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
#if APP_WITH_S5
	} else if (g_mode == CAR_AVOID) {
		avoid_step();		/* uses g_lb / g_rb set above */
#endif
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
#if APP_WITH_S5
		} else if (g_mode == CAR_AVOID) {
			avoid_step();		/* no line data: keep the timing */
#endif
		} else {
			/* nothing this pass */
		}

		while (tk_rcv_mbf(mbf_barcode, &b, TMO_POL) == (INT)sizeof(b)) {
			on_barcode(b);
		}

		/* status line, 2 Hz (none before READY, like integrated_car) */
		if (((W)(t - next_print_ms) >= 0) && s3_is_calibrated()) {
			UB pend = 0U;
			UB bsrc = 0U;
			UW bisr = 0U, bsamp = 0U, bpush = 0U;
			next_print_ms += S2_STATUS_MS;
			if ((W)(t - next_print_ms) >= 0) {
				next_print_ms = t + S2_STATUS_MS;	/* after calibration */
			}
			s3_barcode_diag(&bsrc, &bisr, &bsamp, &bpush);
			tm_printf((UB *)"mode=%u L=%c R=%c err=%4d jn=%u lost=%u pending=%c pwm=%d/%d link=%s"
				  " bar=%c irq%u smp%u q%u\n",
				  (UINT)g_mode, g_lb ? 'B' : 'W', g_rb ? 'B' : 'W',
				  line_err_get_error(le), (UINT)line_err_get_junction(le),
				  (UINT)line_err_get_lost(le),
				  nav_has_pending(&g_nav, &pend) ? (char)('A' + pend) : '-',
				  (g_mode == CAR_FOLLOW) ? g_out_l : 0,
				  (g_mode == CAR_FOLLOW) ? g_out_r : 0,
				  s1_link_name(), (char)bsrc, (UINT)bisr, (UINT)bsamp, (UINT)bpush);
		}
		(void)tk_set_flg(flg_hb, HB_MISSION_CONTROL);
	}
}

/*
 *	task_obstacle.c
 *	Subsystem 5 -- obstacle_task and its statechart. See README.md.
 *
 *	Rules kept (Tribunal 1):
 *	  - Never blocks longer than OA_WAIT_CHUNK_MS (+ one ping) without
 *	    setting its heartbeat bit, so the Safety Monitor never sees a hang.
 *	  - Never touches the motor PWM (motor_task owns it): every move of
 *	    the bypass and line recovery is asked of Mission Control on
 *	    motion_q, and S5 waits for its flg_avoid reply.
 *	  - Sends with TMO_POL; a full queue is counted, not waited on.
 *	  - Sets only its own fault bits; Mission Control clears them.
 *	Kernel tick is 10 ms (CNF_TIMER_PERIOD): every timeout below is
 *	really rounded to the tick.
 */

#include "subsystem5.h"
#include "obstacle_config.h"
#include "obstacle_profile.h"
#include "drv_hcsr04.h"
#include "drv_servo.h"
#include "s5_hw.h"

#define US_PER_MS	(1000U)
#define OA_PROBE_PERIOD_MS	(80U)

#if (OA_PROBE_PIN >= 0)
#define PROBE_INIT()	do { gpio_set_pin((UINT)OA_PROBE_PIN, GPIO_MODE_OUT);	\
			     gpio_set_val((UINT)OA_PROBE_PIN, 0U); } while (0)
#define PROBE_SET(b)	gpio_set_val((UINT)OA_PROBE_PIN, (b) ? 1U : 0U)
#else
#define PROBE_INIT()	do { } while (0)
#define PROBE_SET(b)	do { (void)(b); } while (0)
#endif

//testing stuff
#ifndef OA_SENSOR_PROBE
#define OA_SENSOR_PROBE (0)
#endif

/*
 * Change Requirement 1: small state and counters packed into C bit-fields.
 * Safe because s_ctx is touched by THIS task only: no ISR, no other task,
 * so the bit-field read-modify-writes cannot race. Anything shared is a
 * whole volatile word or goes through a kernel object.
 * Barr-C: bit-fields only in unsigned int.
 */
#define CTX_STATE_BITS		(3U)	/* 8 states                  */
#define CTX_CONFIRM_BITS	(2U)	/* 0..OA_CONFIRM_COUNT (2)   */
#define CTX_NO_ECHO_BITS	(3U)	/* 0..OA_NO_ECHO_LIMIT (5)   */
#define CTX_RETRY_BITS		(2U)	/* 0..OA_MAX_RETRIES (2)     */
#define CTX_PLAN_BITS		(3U)	/* obs_plan_t, 5 values      */

#if (OA_CONFIRM_COUNT >= (1U << CTX_CONFIRM_BITS)) || \
    (OA_NO_ECHO_LIMIT >= (1U << CTX_NO_ECHO_BITS)) || \
    (OA_MAX_RETRIES   >= (1U << CTX_RETRY_BITS))
#error "A tuning limit no longer fits its bit-field: widen it in oa_ctx_t."
#endif

typedef struct
{
	unsigned int	state          : CTX_STATE_BITS;
	unsigned int	confirm_count  : CTX_CONFIRM_BITS;
	unsigned int	no_echo_count  : CTX_NO_ECHO_BITS;
	unsigned int	retries        : CTX_RETRY_BITS;
	unsigned int	plan           : CTX_PLAN_BITS;
	unsigned int	b_sensor_fault : 1;
	uint8_t		fine_start_deg;
	uint8_t		fine_end_deg;
	uint8_t		sidestep_cm;	/* not a bit-field: its address is passed */
	UW		last_trigger_us;
	oa_scan_t	scan;
	oa_profile_t	prof;
} oa_ctx_t;

/* How one motion_q move ended. */
typedef enum
{
	OA_MOVE_DONE = 0,	/* finished as asked                       */
	OA_MOVE_LINE,		/* FWD_TO_LINE: stopped on the line         */
	OA_MOVE_BLOCKED,	/* something within OA_BYPASS_ABORT_CM ahead */
	OA_MOVE_ABORTED,	/* Mission Control stopped the car (ABORT)  */
	OA_MOVE_TIMEOUT		/* no reply in OA_MOVE_TIMEOUT_MS           */
} oa_move_result_t;

/* One leg of the bypass. */
typedef struct
{
	motor_mode_t	mode;
	UB		param;		/* cm or deg */
	BOOL		b_watch;	/* ping ahead while it runs */
} oa_leg_t;

LOCAL oa_ctx_t		s_ctx;			/* task-private            */
LOCAL volatile UW	s_state_pub = 0U;	/* task writes, any reads  */
LOCAL volatile UW	s_dropped_msgs = 0U;	/* task writes, any reads  */

/* Increment without wrapping a narrow bit-field back to 0. */
#define SAT_INC(field, max)	do { if ((field) < (max)) { (field)++; } } while (0)

/* ------------------------------------------------------------ helpers */

LOCAL void oa_heartbeat(void)
{
	(void)tk_set_flg(S5_HB_FLG, S5_HB_BIT);
}

LOCAL void oa_set_fault(UINT fault_bit)
{
	(void)tk_set_flg(S5_FAULT_FLG, fault_bit);	/* atomic in the kernel */
}

/* Sleep in heartbeat-sized chunks. */
LOCAL void oa_sleep_ms(UW ms)
{
	UW chunk;

	while (ms > 0U) {
		chunk = (ms > OA_WAIT_CHUNK_MS) ? OA_WAIT_CHUNK_MS : ms;
		(void)tk_dly_tsk((TMO)chunk);
		oa_heartbeat();
		ms -= chunk;
	}
}

/* Wait for any of 'bits' on flg_avoid, in chunks, up to timeout_ms.
   Returns the bits that arrived (masked to 'bits'), or 0 on timeout. */
LOCAL UINT oa_wait_avoid_evt(UINT bits, UW timeout_ms)
{
	UW	waited = 0U;
	UINT	flgptn = 0U;
	ER	er;

	do {
		er = tk_wai_flg(flg_avoid, bits, TWF_ORW | TWF_BITCLR,
				&flgptn, (TMO)OA_WAIT_CHUNK_MS);
		oa_heartbeat();
		if (er == E_OK) {
			return (flgptn & bits);
		}
		waited += OA_WAIT_CHUNK_MS;
	} while ((er == E_TMOUT) && (waited < timeout_ms));

	return 0U;
}

#if OA_DEBUG
#define OA_DEBUG_PERIOD_MS	(1000U)

LOCAL UW s_dbg_last_us = 0U;	/* task-private */

LOCAL const char * const k_dbg_state[] = { "MONITOR", "WAIT_STOP", "COARSE",
					   "FINE", "PROFILE", "PLAN", "BYPASS",
					   "FAULT" };
LOCAL const char * const k_dbg_ping[]  = { "OK", "NO_ECHO", "BUSY", "TOO_SHORT" };

/* At most one status line per second: tm_printf over USB costs a few ms,
   so printing every ping would upset the 60 ms ping pacing. */
LOCAL void oa_debug_status(hcsr04_status_t status, UB dist_cm)
{
	UW e, g, w;

	if (((s5_time_us() - s_dbg_last_us) / US_PER_MS) < OA_DEBUG_PERIOD_MS) {
		return;
	}
	s_dbg_last_us = s5_time_us();

	drv_hcsr04_diag(&e, &g, &w);
	tm_printf((UB *)"[S5 dbg] state=%s ping=%s dist=%u last_width=%uus "
		  "edges=%u noise=%u bad_in_row=%u\n",
		  k_dbg_state[s_ctx.state & 7U], k_dbg_ping[(UINT)status & 3U],
		  (UINT)dist_cm, (UINT)w, (UINT)e, (UINT)g,
		  (UINT)s_ctx.no_echo_count);
}

/* " 115:255" is at most 8 chars per point, plus the header. */
#define OA_DEBUG_SCAN_BUF	(32U + (8U * OA_MAX_SCAN_POINTS))

LOCAL UB s_dbg_scan_buf[OA_DEBUG_SCAN_BUF];	/* task-private, off the stack */

/* One line per finished sweep: every angle:distance pair (255 = no echo
   within range). Angles > 90 are LEFT when OA_SERVO_LEFT_IS_HIGH. */
LOCAL void oa_debug_scan(void)
{
	UB	*p = s_dbg_scan_buf;
	UB	i;

	p += tm_sprintf(p, (UB *)"[S5 dbg] scan %s:",
			k_dbg_state[s_ctx.state & 7U]);
	for (i = 0U; i < s_ctx.scan.count; i++) {
		p += tm_sprintf(p, (UB *)" %u:%u",
				(UINT)scan_angle(&s_ctx.scan, i),
				(UINT)s_ctx.scan.dist_cm[i]);
	}
	tm_printf((UB *)"%s\n", s_dbg_scan_buf);
}
#endif /* OA_DEBUG */

/* One ping, never faster than the HC-SR04's 60 ms minimum period. Paced
   on the last trigger time, so the period does not drift (Lab 5). */
LOCAL hcsr04_status_t oa_ping(UB *p_dist_cm)
{
	UW const	since_ms = (s5_time_us() - s_ctx.last_trigger_us) / US_PER_MS;
	hcsr04_status_t	status;

	if (since_ms < OA_PING_PERIOD_MS) {
		oa_sleep_ms(OA_PING_PERIOD_MS - since_ms);
	}

	s_ctx.last_trigger_us = s5_time_us();
	status = drv_hcsr04_measure_cm(p_dist_cm);
	oa_heartbeat();

	/* NO_ECHO (ECHO never rose) and BUSY (ECHO already high, i.e. stuck)
	   both mean the sensor is not working: either one, 5 pings in a row,
	   is a fault. A stuck-high ECHO used to reset the count and hide. */
	/* TOO_SHORT (only < OA_DIST_MIN_VALID_CM echoes) counts too: now and
	   then it is noise and is simply ignored, but every ping in a row
	   means something is stuck in front of / against the sensor. */
	if ((status == HCSR04_NO_ECHO) || (status == HCSR04_BUSY) ||
	    (status == HCSR04_TOO_SHORT)) {
		SAT_INC(s_ctx.no_echo_count, OA_NO_ECHO_LIMIT);
		if (s_ctx.no_echo_count >= OA_NO_ECHO_LIMIT) {
#if OA_DEBUG
			/* Printed once, when the fault is first latched. (This
			   used to sit outside the if, so it printed "FAULT" after
			   every HEALTHY ping.) */
			if (s_ctx.b_sensor_fault == 0U) {
				UW e, g, w;

				drv_hcsr04_diag(&e, &g, &w);
				tm_printf((UB *)"[S5 dbg] FAULT: %s %u pings in a row "
					  "(edges=%u noise=%u)\n",
					  (status == HCSR04_BUSY) ? "ECHO stuck HIGH" :
					  (status == HCSR04_TOO_SHORT) ? "echo too short (< min valid cm)"
								      : "no echo",
					  (UINT)OA_NO_ECHO_LIMIT, (UINT)e, (UINT)g);
			}
#endif
			oa_set_fault(S5_FAULT_NO_ECHO);
			s_ctx.b_sensor_fault = 1U;
		}
	} else {
		s_ctx.no_echo_count = 0U;
	}

#if OA_DEBUG
	oa_debug_status(status, *p_dist_cm);
#endif
	return status;
}

LOCAL void oa_servo_move(UB angle_deg)
{
	UB const from_deg = drv_servo_get_angle();

	drv_servo_set_angle(angle_deg);
	oa_sleep_ms(drv_servo_travel_ms(from_deg, angle_deg));
}

LOCAL void oa_send(obs_msg_type_t type, obs_plan_t plan)
{
	obstacle_msg_t	msg;
	uint8_t		buf[OBSTACLE_MSG_SIZE];

	msg.type           = type;
	msg.plan           = plan;
	msg.state          = (uint8_t)s_ctx.state;
	msg.closest_cm     = s_ctx.prof.closest_cm;
	msg.width_cm       = s_ctx.prof.width_cm;
	msg.left_clear_cm  = s_ctx.prof.left_clear_cm;
	msg.right_clear_cm = s_ctx.prof.right_clear_cm;
	msg.sidestep_cm    = s_ctx.sidestep_cm;

	obstacle_msg_pack(&msg, buf);

	if (tk_snd_mbf(S5_OBSTACLE_MBF, buf, (INT)OBSTACLE_MSG_SIZE, TMO_POL) != E_OK) {
		s_dropped_msgs++;	/* single writer: this task */
	}
}

/* Clear a stale reply bit BEFORE asking, so an old reply cannot satisfy us. */
LOCAL void oa_request(UINT reply_bit, obs_msg_type_t type, obs_plan_t plan)
{
	/* ABORT too: one left over from an earlier avoid must not cancel
	   this new request. */
	(void)tk_clr_flg(flg_avoid, ~(reply_bit | AVOID_EVT_ABORT));
	oa_send(type, plan);
}

LOCAL UB oa_clamp_cm(UW cm)
{
	return (cm > OA_MOVE_MAX_CM) ? (UB)OA_MOVE_MAX_CM : (UB)cm;
}

#if OA_DEBUG
LOCAL const char * const k_dbg_move[] = { "STOP", "STEER", "FWD", "BWD", "TURN_L",
					  "TURN_R", "U_TURN", "FWD_TO_LINE" };
#endif

/*
 * Ask Mission Control for one move (motion_q) and wait until it is done.
 * b_watch: keep pinging straight ahead; something within
 * OA_BYPASS_ABORT_CM ends the wait with OA_MOVE_BLOCKED (the caller then
 * sends OBSTACLE, which makes Mission Control stop at once).
 * ABORT is not cleared here: one that arrives between two moves must still
 * end the bypass. oa_request() clears it when a new avoid starts.
 */
LOCAL oa_move_result_t oa_move(motor_mode_t mode, UB param, BOOL b_watch)
{
	motor_cmd_msg_t const	m = motor_cmd_pack(mode, 0U, (int8_t)oa_clamp_cm(param));
	UW const		start_us = s5_time_us();
	UINT const		bits = AVOID_EVT_MOVE_DONE | AVOID_EVT_LINE_FOUND |
				       AVOID_EVT_ABORT;
	UINT			flgptn = 0U;
	UB			dist_cm;
	ER			er;

#if OA_DEBUG
	tm_printf((UB *)"[S5 dbg] move %s %u\n", k_dbg_move[(UINT)mode & 7U], (UINT)param);
#endif
	(void)tk_clr_flg(flg_avoid, ~(AVOID_EVT_MOVE_DONE | AVOID_EVT_LINE_FOUND));
	if (tk_snd_mbf(S5_MOTION_MBF, &m, (INT)sizeof(m), TMO_POL) != E_OK) {
		s_dropped_msgs++;
		return OA_MOVE_TIMEOUT;
	}

	for (;;) {
		if (b_watch) {
			dist_cm = (UB)OA_DIST_OUT_OF_RANGE;
			if ((oa_ping(&dist_cm) == HCSR04_OK) && (dist_cm <= OA_BYPASS_ABORT_CM)) {
				s_ctx.prof.closest_cm = dist_cm;
				return OA_MOVE_BLOCKED;
			}
			if (s_ctx.b_sensor_fault != 0U) {
				return OA_MOVE_ABORTED;	/* main loop goes to FAULT */
			}
		}
		/* oa_ping() already paces the loop at 60 ms when watching. */
		er = tk_wai_flg(flg_avoid, bits, TWF_ORW | TWF_BITCLR, &flgptn,
				b_watch ? TMO_POL : (TMO)OA_WAIT_CHUNK_MS);
		oa_heartbeat();
		if (er == E_OK) {
			if ((flgptn & AVOID_EVT_ABORT) != 0U) {
				return OA_MOVE_ABORTED;
			}
			return ((flgptn & AVOID_EVT_LINE_FOUND) != 0U) ? OA_MOVE_LINE
								       : OA_MOVE_DONE;
		}
		if (((s5_time_us() - start_us) / US_PER_MS) >= OA_MOVE_TIMEOUT_MS) {
			return OA_MOVE_TIMEOUT;
		}
	}
}

/* Next state after a move that did not end as planned. */
LOCAL s5_state_t oa_move_failed(oa_move_result_t r)
{
	switch (r) {
	case OA_MOVE_BLOCKED:
		/* Something new, close, ahead: stop and rescan from here. */
		SAT_INC(s_ctx.retries, OA_MAX_RETRIES);
		oa_request(AVOID_EVT_MOTORS_STOPPED, OBS_MSG_OBSTACLE, OBS_PLAN_STOP);
		return S5_STATE_WAIT_STOP;

	case OA_MOVE_TIMEOUT:
#if OA_DEBUG
		tm_printf((UB *)"[S5 dbg] FAULT: Mission Control did not finish a move in %u ms\n",
			  (UINT)OA_MOVE_TIMEOUT_MS);
#endif
		oa_set_fault(S5_FAULT_TIMEOUT);
		return S5_STATE_FAULT;

	case OA_MOVE_ABORTED:
	default:
		/* Mission Control stopped the car itself: not a fault. */
		return S5_STATE_MONITOR;
	}
}

/* Sweep start, start+step, ... up to end (inclusive) into s_ctx.scan. */
LOCAL void oa_sweep(UB start_deg, UB step_deg, UB end_deg)
{
	UB	dist_cm;
	UH	angle;		/* 16 bits: 'angle += step' cannot wrap past 255 */

	s_ctx.scan.start_deg = start_deg;
	s_ctx.scan.step_deg  = step_deg;
	s_ctx.scan.count     = 0U;

	for (angle = start_deg;
	     (angle <= end_deg) && (s_ctx.scan.count < OA_MAX_SCAN_POINTS);
	     angle += step_deg) {
		dist_cm = (UB)OA_DIST_OUT_OF_RANGE;
		oa_servo_move((UB)angle);
		if (oa_ping(&dist_cm) == HCSR04_TOO_SHORT) {
			/* Noise: one more try, so a bad ping does not leave a
			   false "open" (255) point in the profile. */
			(void)oa_ping(&dist_cm);
		}

		s_ctx.scan.dist_cm[s_ctx.scan.count] = dist_cm;
		s_ctx.scan.count++;
	}

#if OA_DEBUG
	oa_debug_scan();
#endif
}

/* ------------------------------------------------------------ states */

LOCAL s5_state_t oa_state_monitor(void)
{
	UB dist_cm = (UB)OA_DIST_OUT_OF_RANGE;

	if ((oa_ping(&dist_cm) == HCSR04_OK) && (dist_cm <= OA_STOP_DIST_CM)) {
		SAT_INC(s_ctx.confirm_count, OA_CONFIRM_COUNT);
		if (s_ctx.confirm_count >= OA_CONFIRM_COUNT) {
			s_ctx.prof.closest_cm = dist_cm;
			oa_request(AVOID_EVT_MOTORS_STOPPED, OBS_MSG_OBSTACLE, OBS_PLAN_STOP);
			return S5_STATE_WAIT_STOP;
		}
	} else {
		s_ctx.confirm_count = 0U;
	}
	return S5_STATE_MONITOR;
}

LOCAL s5_state_t oa_state_wait_stop(void)
{
	UINT const got = oa_wait_avoid_evt(AVOID_EVT_MOTORS_STOPPED | AVOID_EVT_ABORT,
					   OA_STOP_ACK_TIMEOUT_MS);

	if ((got & AVOID_EVT_ABORT) != 0U) {
		/* Mission Control stopped the car itself: not a fault. */
		return S5_STATE_MONITOR;
	}
	if ((got & AVOID_EVT_MOTORS_STOPPED) != 0U) {
		return S5_STATE_COARSE_SCAN;
	}
#if OA_DEBUG
	tm_printf((UB *)"[S5 dbg] FAULT: Mission Control did not answer MOTORS_STOPPED in %u ms\n",
		  (UINT)OA_STOP_ACK_TIMEOUT_MS);
#endif
	oa_set_fault(S5_FAULT_TIMEOUT);
	return S5_STATE_FAULT;
}

LOCAL s5_state_t oa_state_coarse_scan(void)
{
	oa_sweep(OA_COARSE_START_DEG, OA_COARSE_STEP_DEG,
		 (UB)(OA_COARSE_START_DEG + ((OA_COARSE_COUNT - 1U) * OA_COARSE_STEP_DEG)));

	if (profile_fine_window(&s_ctx.scan, &s_ctx.fine_start_deg, &s_ctx.fine_end_deg)) {
		return S5_STATE_FINE_SCAN;
	}
	oa_send(OBS_MSG_CLEAR, OBS_PLAN_CONTINUE);	/* obstacle has gone */
	return S5_STATE_MONITOR;
}

LOCAL s5_state_t oa_state_fine_scan(void)
{
	/* TODO: alternate sweep direction each scan to save servo travel. */
	oa_sweep(s_ctx.fine_start_deg, (UB)OA_FINE_STEP_DEG, s_ctx.fine_end_deg);
	return S5_STATE_PROFILE;
}

LOCAL s5_state_t oa_state_profile(void)
{
	profile_build(&s_ctx.scan, &s_ctx.prof);

	if (!s_ctx.prof.b_found) {
		oa_send(OBS_MSG_CLEAR, OBS_PLAN_CONTINUE);
		return S5_STATE_MONITOR;
	}
	return S5_STATE_PLAN;
}

LOCAL s5_state_t oa_state_plan(void)
{
	obs_plan_t const plan = profile_plan(&s_ctx.prof, (uint8_t)s_ctx.retries,
					     &s_ctx.sidestep_cm);

	s_ctx.plan = ((unsigned int)plan) & ((1U << CTX_PLAN_BITS) - 1U);

	switch (plan) {
	case OBS_PLAN_LEFT:
	case OBS_PLAN_RIGHT:
		oa_send(OBS_MSG_PLAN, plan);		/* report only */
		return S5_STATE_BYPASS;

	case OBS_PLAN_REVERSE:
	{
		oa_move_result_t r;

		SAT_INC(s_ctx.retries, OA_MAX_RETRIES);
		oa_send(OBS_MSG_PLAN, plan);		/* report only */
		r = oa_move(MCMD_BWD_DIST, (UB)OA_REVERSE_CM, FALSE);
		return (r == OA_MOVE_DONE) ? S5_STATE_COARSE_SCAN : oa_move_failed(r);
	}

	case OBS_PLAN_CONTINUE:
		oa_send(OBS_MSG_CLEAR, OBS_PLAN_CONTINUE);
		return S5_STATE_MONITOR;

	case OBS_PLAN_STOP:
	default:
		oa_send(OBS_MSG_PLAN, OBS_PLAN_STOP);
#if OA_DEBUG
		tm_printf((UB *)"[S5 dbg] FAULT: no route (both sides blocked, retries used)\n");
#endif
		oa_set_fault(S5_FAULT_NO_ROUTE);
		return S5_STATE_FAULT;
	}
}

/*
 * Stage 4 (drive the bypass) + Stage 5 (line recovery), on side D:
 *   1 turn 45 deg toward D            leave the line
 *   2 forward sidestep * sqrt(2)      ends sidestep cm off the line
 *   3 turn 45 deg back                parallel to the line
 *   4 forward to past the obstacle    (closest - sidestep) + OA_PASS_EXTRA_CM
 *   5 turn 45 deg back toward the line
 *   6 forward until a front IR sensor sees the line (up to 2x leg 2)
 *   7 turn 45 deg toward D            heading along the line again
 * then CLEAR hands the car back to line following. If leg 6 misses the
 * line, one more 45 deg turn (square to the line) and a second search of
 * OA_SEEK_MAX_CM; leg 7 is then 90 deg. Forward legs keep pinging ahead.
 */
LOCAL s5_state_t oa_state_bypass(void)
{
	BOOL const		b_left = (s_ctx.plan == (unsigned int)OBS_PLAN_LEFT) ? TRUE : FALSE;
	motor_mode_t const	turn_out  = b_left ? MCMD_TURN_LEFT  : MCMD_TURN_RIGHT;
	motor_mode_t const	turn_back = b_left ? MCMD_TURN_RIGHT : MCMD_TURN_LEFT;
	UW const		side_cm = (s_ctx.sidestep_cm == (uint8_t)OA_DIST_OUT_OF_RANGE)
					? OA_DEFAULT_SIDESTEP_CM : (UW)s_ctx.sidestep_cm;
	UW const		diag_cm = (side_cm * 141U) / 100U;	/* x sqrt(2) */
	UW const		ahead_cm = ((UW)s_ctx.prof.closest_cm > side_cm)
					 ? ((UW)s_ctx.prof.closest_cm - side_cm) : 0U;
	oa_leg_t		legs[5];
	oa_move_result_t	r = OA_MOVE_DONE;
	UB			align_deg = (UB)OA_TURN_OUT_DEG;
	UINT			i;

	legs[0].mode = turn_out;      legs[0].param = (UB)OA_TURN_OUT_DEG; legs[0].b_watch = FALSE;
	legs[1].mode = MCMD_FWD_DIST; legs[1].param = oa_clamp_cm(diag_cm); legs[1].b_watch = TRUE;
	legs[2].mode = turn_back;     legs[2].param = (UB)OA_TURN_OUT_DEG; legs[2].b_watch = FALSE;
	legs[3].mode = MCMD_FWD_DIST; legs[3].param = oa_clamp_cm(ahead_cm + OA_PASS_EXTRA_CM);
	legs[3].b_watch = TRUE;
	legs[4].mode = turn_back;     legs[4].param = (UB)OA_TURN_OUT_DEG; legs[4].b_watch = FALSE;

	/* Stage 4: around the obstacle. */
	for (i = 0U; i < (sizeof(legs) / sizeof(legs[0])); i++) {
		r = oa_move(legs[i].mode, legs[i].param, legs[i].b_watch);
		if (r != OA_MOVE_DONE) {
			return oa_move_failed(r);
		}
	}

	/* Stage 5: line recovery. */
	r = oa_move(MCMD_FWD_TO_LINE, oa_clamp_cm(diag_cm * OA_SEEK_FACTOR), TRUE);
	if (r == OA_MOVE_DONE) {
		/* Missed it: turn square to the line and look once more. */
		r = oa_move(turn_back, (UB)OA_TURN_OUT_DEG, FALSE);
		if (r == OA_MOVE_DONE) {
			r = oa_move(MCMD_FWD_TO_LINE, (UB)OA_SEEK_MAX_CM, TRUE);
		}
		align_deg = (UB)(2U * OA_TURN_OUT_DEG);
		if (r == OA_MOVE_DONE) {
#if OA_DEBUG
			tm_printf((UB *)"[S5 dbg] FAULT: line not found after the bypass\n");
#endif
			oa_set_fault(S5_FAULT_NO_LINE);
			return S5_STATE_FAULT;
		}
	}
	if (r != OA_MOVE_LINE) {
		return oa_move_failed(r);
	}

	/* On the line: turn back along it and hand control back. */
	r = oa_move(turn_out, align_deg, FALSE);
	if (r != OA_MOVE_DONE) {
		return oa_move_failed(r);
	}
	oa_send(OBS_MSG_CLEAR, OBS_PLAN_CONTINUE);
	return S5_STATE_MONITOR;
}

LOCAL s5_state_t oa_state_fault(void)
{
	if (oa_wait_avoid_evt(AVOID_EVT_RESUME, OA_WAIT_CHUNK_MS) != 0U) {
		s_ctx.b_sensor_fault = 0U;
		s_ctx.no_echo_count  = 0U;
		return S5_STATE_MONITOR;
	}
	return S5_STATE_FAULT;
}

/* ------------------------------------------------------------ task */

/* Entry actions for each state. */
LOCAL void oa_enter(s5_state_t next)
{
	s_ctx.state = ((unsigned int)next) & ((1U << CTX_STATE_BITS) - 1U);
	s_state_pub = (UW)next;

	switch (next) {
	case S5_STATE_WAIT_STOP:
		PROBE_SET(TRUE);		/* rising edge = obstacle confirmed */
		break;

	case S5_STATE_MONITOR:
		PROBE_SET(FALSE);
		s_ctx.confirm_count = 0U;
		s_ctx.retries       = 0U;
		s_ctx.sidestep_cm   = (uint8_t)OA_DIST_OUT_OF_RANGE;
		profile_reset(&s_ctx.prof);
		oa_servo_move(OA_SERVO_CENTRE_DEG);
		break;

	case S5_STATE_BYPASS:
		PROBE_SET(FALSE);		/* falling edge = plan sent */
		oa_servo_move(OA_SERVO_CENTRE_DEG);
		break;

	case S5_STATE_FAULT:
		PROBE_SET(FALSE);
		/* Forget any old RESUME, so only a resume sent AFTER this fault
		   (Mission Control: START / GP20) can clear it. */
		(void)tk_clr_flg(flg_avoid, ~(UINT)AVOID_EVT_RESUME);
		oa_send(OBS_MSG_FAULT, OBS_PLAN_STOP);
		break;

	default:
		break;
	}
}

EXPORT void obstacle_task(INT stacd, void *exinf)
{
	s5_state_t	next;
	ER		er;

	(void)stacd;
	(void)exinf;

	/* Hardware set-up runs HERE, on core 0, so the echo interrupt is
	   enabled on the same core as this task. */
	er = drv_hcsr04_init();
	CHECK(er, "drv_hcsr04_init");
	drv_servo_init();
	PROBE_INIT();

	#if OA_SENSOR_PROBE
	{
		UW last_us = s5_time_us();

		for (;;) {
			UB		d = (UB)OA_DIST_OUT_OF_RANGE;
			UW const	since_ms = (s5_time_us() - last_us) / 1000U;
			hcsr04_status_t	st;

			if (since_ms < OA_PROBE_PERIOD_MS) {
				(void)tk_dly_tsk((TMO)(OA_PROBE_PERIOD_MS - since_ms));
			}
			last_us = s5_time_us();
			st = drv_hcsr04_measure_cm(&d);
			{
				UW e, g, w;

				drv_hcsr04_diag(&e, &g, &w);
				/* st: 0 OK, 1 NO_ECHO, 2 BUSY, 3 TOO_SHORT (noise) */
				tm_printf((UB *)"[probe] st=%d dist=%u width=%uus edges=%u noise=%u\n",
					  (INT)st, (UINT)d, (UINT)w, (UINT)e, (UINT)g);
			}
			oa_heartbeat();
		}
	}
#endif

	/* Do not ping until Mission Control is draining obstacle_q. It first
	   waits up to 10 s for the USB console; an obstacle reported before
	   then got no MOTORS_STOPPED within OA_STOP_ACK_TIMEOUT_MS and
	   latched a false TIMEOUT fault at boot. Heartbeat every chunk. */
	while (oa_wait_avoid_evt(AVOID_EVT_MC_READY, OA_WAIT_CHUNK_MS) == 0U) {
		/* Waiting for Mission Control. */
	}

	s_ctx.last_trigger_us = s5_time_us();
	profile_reset(&s_ctx.prof);
	oa_enter(S5_STATE_MONITOR);

	for (;;) {
		switch ((s5_state_t)s_ctx.state) {
		case S5_STATE_MONITOR:     next = oa_state_monitor();     break;
		case S5_STATE_WAIT_STOP:   next = oa_state_wait_stop();   break;
		case S5_STATE_COARSE_SCAN: next = oa_state_coarse_scan(); break;
		case S5_STATE_FINE_SCAN:   next = oa_state_fine_scan();   break;
		case S5_STATE_PROFILE:     next = oa_state_profile();     break;
		case S5_STATE_PLAN:        next = oa_state_plan();        break;
		case S5_STATE_BYPASS:      next = oa_state_bypass();      break;
		case S5_STATE_FAULT:
		default:                   next = oa_state_fault();       break;
		}

		if ((s_ctx.b_sensor_fault != 0U) && (next != S5_STATE_FAULT)) {
			next = S5_STATE_FAULT;
		}
		if (next != (s5_state_t)s_ctx.state) {
			oa_enter(next);
		}
	}
}

EXPORT s5_state_t s5_get_state(void)
{
	return (s5_state_t)s_state_pub;
}

EXPORT UW s5_dropped_msgs(void)
{
	return s_dropped_msgs;
}

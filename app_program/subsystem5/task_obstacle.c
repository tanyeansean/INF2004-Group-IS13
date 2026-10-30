/*
 *	task_obstacle.c
 *	Subsystem 5 -- obstacle_task and its statechart. See README.md.
 *
 *	Rules kept (Tribunal 1):
 *	  - Never blocks longer than OA_WAIT_CHUNK_MS (+ one ping) without
 *	    setting its heartbeat bit, so the Safety Monitor never sees a hang.
 *	  - Never drives the motors: asks Mission Control via obstacle_q and
 *	    waits for flg_avoid bits.
 *	  - Sends with TMO_POL; a full obstacle_q is counted, not waited on.
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

#if (OA_PROBE_PIN >= 0)
#define PROBE_INIT()	do { gpio_set_pin((UINT)OA_PROBE_PIN, GPIO_MODE_OUT);	\
			     gpio_set_val((UINT)OA_PROBE_PIN, 0U); } while (0)
#define PROBE_SET(b)	gpio_set_val((UINT)OA_PROBE_PIN, (b) ? 1U : 0U)
#else
#define PROBE_INIT()	do { } while (0)
#define PROBE_SET(b)	do { (void)(b); } while (0)
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
	unsigned int	b_reversing    : 1;
	unsigned int	b_sensor_fault : 1;
	uint8_t		fine_start_deg;
	uint8_t		fine_end_deg;
	uint8_t		sidestep_cm;	/* not a bit-field: its address is passed */
	UW		last_trigger_us;
	UW		bypass_start_us;
	oa_scan_t	scan;
	oa_profile_t	prof;
} oa_ctx_t;

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

/* Wait for any of 'bits' on flg_avoid, in chunks, up to timeout_ms. */
LOCAL BOOL oa_wait_avoid_evt(UINT bits, UW timeout_ms)
{
	UW	waited = 0U;
	UINT	flgptn = 0U;
	ER	er;

	do {
		er = tk_wai_flg(flg_avoid, bits, TWF_ORW | TWF_BITCLR,
				&flgptn, (TMO)OA_WAIT_CHUNK_MS);
		oa_heartbeat();
		if (er == E_OK) {
			return TRUE;
		}
		waited += OA_WAIT_CHUNK_MS;
	} while ((er == E_TMOUT) && (waited < timeout_ms));

	return FALSE;
}

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

	if (status == HCSR04_NO_ECHO) {
		SAT_INC(s_ctx.no_echo_count, OA_NO_ECHO_LIMIT);
		if (s_ctx.no_echo_count >= OA_NO_ECHO_LIMIT) {
			oa_set_fault(S5_FAULT_NO_ECHO);
			s_ctx.b_sensor_fault = 1U;
		}
	} else {
		s_ctx.no_echo_count = 0U;
	}
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
	(void)tk_clr_flg(flg_avoid, ~reply_bit);
	oa_send(type, plan);
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
		(void)oa_ping(&dist_cm);

		s_ctx.scan.dist_cm[s_ctx.scan.count] = dist_cm;
		s_ctx.scan.count++;
	}
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
	UW const timeout_ms = (s_ctx.b_reversing != 0U) ? OA_REVERSE_TIMEOUT_MS
							: OA_STOP_ACK_TIMEOUT_MS;

	if (oa_wait_avoid_evt(AVOID_EVT_MOTORS_STOPPED, timeout_ms)) {
		s_ctx.b_reversing = 0U;
		return S5_STATE_COARSE_SCAN;
	}
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
		oa_request(AVOID_EVT_LINE_REACQUIRED, OBS_MSG_PLAN, plan);
		return S5_STATE_BYPASS;

	case OBS_PLAN_REVERSE:
		SAT_INC(s_ctx.retries, OA_MAX_RETRIES);
		s_ctx.b_reversing = 1U;
		oa_request(AVOID_EVT_MOTORS_STOPPED, OBS_MSG_PLAN, plan);
		return S5_STATE_WAIT_STOP;

	case OBS_PLAN_CONTINUE:
		oa_send(OBS_MSG_CLEAR, OBS_PLAN_CONTINUE);
		return S5_STATE_MONITOR;

	case OBS_PLAN_STOP:
	default:
		oa_send(OBS_MSG_PLAN, OBS_PLAN_STOP);
		oa_set_fault(S5_FAULT_NO_ROUTE);
		return S5_STATE_FAULT;
	}
}

LOCAL s5_state_t oa_state_bypass(void)
{
	UB	dist_cm = (UB)OA_DIST_OUT_OF_RANGE;
	UW	elapsed_ms = (s5_time_us() - s_ctx.bypass_start_us) / US_PER_MS;
	UINT	flgptn = 0U;
	ER	er;

	if (elapsed_ms >= OA_BYPASS_TIMEOUT_MS) {
		oa_set_fault(S5_FAULT_TIMEOUT);
		return S5_STATE_FAULT;
	}

	/* Something new straight ahead while bypassing: stop and rescan. */
	if ((oa_ping(&dist_cm) == HCSR04_OK) && (dist_cm <= OA_BYPASS_ABORT_CM)) {
		SAT_INC(s_ctx.retries, OA_MAX_RETRIES);
		s_ctx.prof.closest_cm = dist_cm;
		oa_request(AVOID_EVT_MOTORS_STOPPED, OBS_MSG_OBSTACLE, OBS_PLAN_STOP);
		return S5_STATE_WAIT_STOP;
	}

	er = tk_wai_flg(flg_avoid, AVOID_EVT_LINE_REACQUIRED, TWF_ORW | TWF_BITCLR,
			&flgptn, (TMO)OA_BYPASS_POLL_MS);
	oa_heartbeat();

	if (er == E_OK) {
		oa_send(OBS_MSG_CLEAR, OBS_PLAN_CONTINUE);
		return S5_STATE_MONITOR;
	}
	return S5_STATE_BYPASS;
}

LOCAL s5_state_t oa_state_fault(void)
{
	if (oa_wait_avoid_evt(AVOID_EVT_RESUME, OA_WAIT_CHUNK_MS)) {
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
		s_ctx.bypass_start_us = s5_time_us();
		oa_servo_move(OA_SERVO_CENTRE_DEG);
		break;

	case S5_STATE_FAULT:
		PROBE_SET(FALSE);
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

/*
 *	task_telemetry.c
 *	Subsystem 1 -- Telemetry task (lowest application priority, NFR5).
 *
 *	Every 10 ms: applies Mission Control's status_q messages to the packed
 *	snapshot (telem_set_*, telem_event_*), then comms_poll() (commands,
 *	map, acks, heartbeat) and telem_poll() (live data 5 Hz, events).
 *	The telem_*() functions are not thread-safe; only this task calls
 *	them, which is why other tasks send their data on status_q.
 *
 *	Fault mask in live data (16 bits): bits 0-11 = the team's fault table
 *	(flg_faults, shared_types.h), read here without changing it; bits
 *	12-14 = the hung task's number from the Safety Monitor (1 Motor,
 *	2 Mission, 3 Line, 4 Barcode, 5 Terrain, 6 Obstacle, 7 Telemetry,
 *	0 = none); bit 15 = WiFi loss (telemetry.c).
 */

#include "subsystem1.h"
#include "comms.h"
#include "../subsystem2/subsystem2.h"	/* IF-04: mbf_cmd */
#include "../subsystem3/subsystem3.h"	/* s3_is_calibrated() */
#include "../subsystem4/subsystem4.h"	/* s4_report(): IMU / humps (added 13 Nov by Jin) */

#define S1_PERIOD_MS		10
#define S1_STATUS_PER_POLL	16

/* Added 19 Nov by Jin: see update_motion() and on_state() below. */
LOCAL W		g_dist_base_mm;		/* car-centre distance at START */
LOCAL BOOL	g_b_running = FALSE;	/* last state was a running one */

/* Laptop command -> Mission Control's cmd_q (comms.h: runs in this task). */
LOCAL bool on_command(comms_cmd_t const *p_cmd)
{
	switch (p_cmd->type) {
	case CMD_RESET_PEAK:
		telem_reset_peak();		/* Telemetry's own data */
		s4_new_run();			/* and Terrain Monitor's highest hump */
		return true;
	case CMD_SUMMARY:
		telem_publish_summary();	/* run totals so far */
		return true;
	case CMD_CALIBRATE:
		/* Dashboard "Calibrate now" (added 19 Nov by Jin): only while
		   the car is stopped, never mid-run.  Subsystem 3 does it. */
		if (g_b_running) {
			tm_printf((UB *)"[car] calibrate ignored - stop the car first\n");
			return false;		/* ack: rejected */
		}
		/* Already calibrating (at power-on, or a press before): a
		   second request would start another round straight after,
		   out of step with the dashboard's timer (car test 19 Nov). */
		if (!s3_is_calibrated()) {
			tm_printf((UB *)"[car] calibrate ignored - still calibrating\n");
			return false;		/* ack: rejected */
		}
		s3_request_recalibrate();
		return true;
	case CMD_START:
		if (!s3_is_calibrated()) {
			tm_printf((UB *)"[car] start ignored - still calibrating\n");
			return false;		/* ack: rejected */
		}
		break;
	default:
		break;
	}
	return (tk_snd_mbf(mbf_cmd, (void *)p_cmd, (INT)sizeof(*p_cmd), TMO_POL) == E_OK);
}

/* Team fault table + Safety Monitor, for the laptop (see the top). */
LOCAL void update_faults(void)
{
	T_RFLG	rflg;
	UW	faults = 0U;

	if (tk_ref_flg(flg_faults, &rflg) == E_OK) {
		faults = rflg.flgptn & 0x0FFFU;
	}
	faults |= (s1_safety_report_task() & 0x7U) << 12;
	telem_set_faults((uint16_t)faults);
}

/* Subsystem 4 -> telemetry (added 13 Nov by Jin).  S4 publishes one 32-bit
   word (s4_report(), single writer, so reading it here needs no lock).
   Live data: tilt, turn rate, motion event.  Event: one hump event each
   time S4's hump counter moves on, with that hump's height. */
LOCAL void update_imu(void)
{
	static UINT	humps_seen = 0U;
	static BOOL	b_started = FALSE;
	UW const	r = s4_report();
	UINT const	humps = s4_report_humps(r);

	telem_set_imu(s4_report_tilt_deg(r), (float)s4_report_turn_dps(r),
		      (imu_event_t)s4_report_motion(r));
	if (!b_started) {
		b_started = TRUE;		/* don't replay humps from before */
		humps_seen = humps;
	} else if (humps != humps_seen) {
		humps_seen = humps;
		telem_event_hump((float)s4_report_last_hump_mm(r) / 10.0f);
	} else {
		/* no new hump */
	}
}

/* Subsystem 2 odometry -> telemetry (added 19 Nov by Jin).  Nothing
   called telem_set_motion(), so the dashboard showed distance, speed and
   encoders as 0 while the car drove (Wi-Fi test 19 Nov, ramp run).
   Speeds and counts come from s2_odom_get() (motor_task, ~20 ms);
   distance is the car centre's distance since START, so the hump event
   and the run summary say how far into the run things happened. */
LOCAL W odom_centre_mm(s2_odom_t const *p)
{
	return (p->dist_mm_l + p->dist_mm_r) / 2;
}

LOCAL void update_motion(void)
{
	s2_odom_t	o;
	W		d_mm;

	s2_odom_get(&o);
	if (o.updates == 0U) {
		return;			/* motor task not started yet */
	}
	d_mm = odom_centre_mm(&o) - g_dist_base_mm;
	if (d_mm < 0) {
		d_mm = 0;		/* backed up past the start (bypass) */
	}
	telem_set_motion((float)o.speed_mms_l / 10.0f, (float)o.speed_mms_r / 10.0f,
			 (int32_t)o.count_l, (int32_t)o.count_r, (float)d_mm / 10.0f);
}

/* Called for every state change, before telem_set_state() sees it. */
LOCAL void on_state(robot_state_t st)
{
	BOOL const	b_run = (st != ROBOT_IDLE) && (st != ROBOT_STOPPED) &&
				(st != ROBOT_ERROR);

	if (b_run && !g_b_running) {
		s2_odom_t	o;

		/* A new run: distance counts from here.  Also re-read the
		   fault table now: START has just cleared it, and the run
		   summary takes its "faults seen" from the live faults at
		   this moment (it showed a lift from BEFORE Start, 19 Nov). */
		s2_odom_get(&o);
		g_dist_base_mm = odom_centre_mm(&o);
		update_faults();
		update_motion();
	}
	g_b_running = b_run;
}

/* S5 scan result -> event/obstacle (added 19 Nov by Jin; see
   s1_post_obstacle()).  obs_plan_t and avoid_action_t use the same order:
   STOP, CONTINUE, LEFT, RIGHT, REVERSE.  S5 sends no angle: 90 (ahead). */
LOCAL void update_obstacle(void)
{
	s1_obst_t		m;
	obstacle_profile_t	p;

	while (tk_rcv_mbf(mbf_s1_obst, &m, TMO_POL) == (INT)sizeof(m)) {
		p.distance_cm    = (float)m.closest_cm;	/* 255 = out of range */
		p.angle_deg      = 90.0f;
		p.width_cm       = (m.width_cm == 255U) ? 0.0f : (float)m.width_cm;
		p.clear_left_cm  = (float)m.left_cm;
		p.clear_right_cm = (float)m.right_cm;
		p.action         = (m.plan <= (UB)AVOID_REVERSE) ? (avoid_action_t)m.plan
								  : AVOID_STOP;
		telem_event_obstacle(&p);
	}
}

LOCAL INT sign7(UH v)
{
	return ((v & 0x40U) != 0U) ? (INT)v - 128 : (INT)v;
}

LOCAL void apply_status(UH m)
{
	UH const	type = (UH)(m & 0x03U);
	char		letter = '?';
	UH		idx = (UH)((m >> 2) & 0x1FU);

	if (idx < 26U) {
		letter = (char)('A' + idx);
	}
	switch (type) {
	case S1_STATUS_STATE:
		on_state((robot_state_t)((m >> 2) & 0x0FU));	/* Jin, 19 Nov */
		telem_set_state((robot_state_t)((m >> 2) & 0x0FU));
		break;
	case S1_STATUS_LINE:
		telem_set_line((uint8_t)((m >> 2) & 0x03U),
			       (int16_t)sign7((UH)((m >> 4) & 0x7FU)),
			       ((m >> 11) & 0x01U) != 0U);
		break;
	case S1_STATUS_BARCODE:
		telem_event_barcode(letter, (telem_move_t)((m >> 7) & 0x07U));
		break;
	default: /* S1_STATUS_JUNCTION */
		telem_event_junction(letter, (telem_move_t)((m >> 7) & 0x07U));
		break;
	}
}

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	UH	m;
	INT	i;
	bool	b_ok;

	(void)stacd; (void)exinf;

	b_ok = comms_init();
	telem_init();				/* also registers the map handler */
	comms_set_cmd_handler(on_command);
	telem_set_state(ROBOT_IDLE);
	if (!b_ok) {
		tm_printf((UB *)"WiFi/MQTT off - running WITHOUT MQTT (GP20 button only)\n");
	}

	for (;;) {
		for (i = 0; (i < S1_STATUS_PER_POLL) &&
			    (tk_rcv_mbf(mbf_status, &m, TMO_POL) == (INT)sizeof(m)); i++) {
			apply_status(m);
		}
		update_faults();
		update_imu();
		update_motion();		/* Jin, 19 Nov */
		update_obstacle();		/* Jin, 19 Nov */
		comms_poll();
		telem_poll();
		(void)tk_set_flg(flg_hb, HB_TELEMETRY);
		tk_dly_tsk(S1_PERIOD_MS);
	}
}

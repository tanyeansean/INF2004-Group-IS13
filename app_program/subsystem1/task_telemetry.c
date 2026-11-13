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
		comms_poll();
		telem_poll();
		(void)tk_set_flg(flg_hb, HB_TELEMETRY);
		tk_dly_tsk(S1_PERIOD_MS);
	}
}

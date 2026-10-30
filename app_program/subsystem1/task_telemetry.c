/*
 *	task_telemetry.c
 *	Subsystem 1 -- Telemetry task (lowest application priority, NFR5).
 *
 *	Every 10 ms: applies Mission Control's status_q messages to the packed
 *	snapshot (telem_set_*, telem_event_*), then comms_poll() (commands,
 *	map, acks, heartbeat) and telem_poll() (live data 5 Hz, events).
 *	The telem_*() functions are not thread-safe; only this task calls
 *	them, which is why other tasks send their data on status_q.
 */

#include "subsystem1.h"
#include "comms.h"
#include "../subsystem2/subsystem2.h"	/* IF-04: mbf_cmd */
#include "../subsystem3/subsystem3.h"	/* s3_is_calibrated() */

#define S1_PERIOD_MS		10
#define S1_STATUS_PER_POLL	16

/* Laptop command -> Mission Control's cmd_q (comms.h: runs in this task). */
LOCAL bool on_command(comms_cmd_t const *p_cmd)
{
	switch (p_cmd->type) {
	case CMD_RESET_PEAK:
		telem_reset_peak();		/* Telemetry's own data */
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
		comms_poll();
		telem_poll();
		(void)tk_set_flg(flg_hb, HB_TELEMETRY);
		tk_dly_tsk(S1_PERIOD_MS);
	}
}

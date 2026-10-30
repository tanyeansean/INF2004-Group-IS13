/*
 *	subsystem2.c
 *	Subsystem 2 -- Mission Control + Motor Control: objects and tasks.
 */

#include "subsystem2.h"

EXPORT ID	mbf_line_err;	/* IF-01, owned by S2 (the receiver) */
EXPORT ID	mbf_barcode;	/* IF-02 */
EXPORT ID	mbf_cmd;	/* IF-04 */
EXPORT ID	mbf_motor;	/* Mission Control -> Motor Control (private) */
EXPORT ID	mbf_obstacle;	/* IF-06, owned by S2 (the receiver) */
EXPORT ID	flg_faults;	/* shared fault flag, cleared only by S2 */

/* Defined here, set up before any task starts; written only by Telemetry
   with nav_map_apply(), read by Mission Control (nav_cmd.h). */
EXPORT nav_map_t	g_nav_map;

#define S2_LINE_MSGS	(8U)	/* 40 ms of line data */
#define S2_BARCODE_MSGS	(4U)
#define S2_CMD_MSGS	(4U)
#define S2_MOTOR_MSGS	(4U)
#define S2_OBST_MSGS	(4U)	/* S5 sends at most ~2 per scan */

LOCAL ID make_mbf(UW msg_size, UW msgs)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)((msg_size + 8U) * msgs);
	cmbf.maxmsz = (INT)msg_size;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem2_init(void)
{
	drv_motor_init();		/* motors stopped first thing */
	nav_map_init(&g_nav_map);

	mbf_line_err = make_mbf(sizeof(line_err_t), S2_LINE_MSGS);
	if (mbf_line_err < E_OK) {
		return mbf_line_err;
	}
	mbf_barcode = make_mbf(sizeof(barcode_t), S2_BARCODE_MSGS);
	if (mbf_barcode < E_OK) {
		return mbf_barcode;
	}
	mbf_cmd = make_mbf(sizeof(comms_cmd_t), S2_CMD_MSGS);
	if (mbf_cmd < E_OK) {
		return mbf_cmd;
	}
	mbf_motor = make_mbf(sizeof(UW), S2_MOTOR_MSGS);
	if (mbf_motor < E_OK) {
		return mbf_motor;
	}
	mbf_obstacle = make_mbf(OBSTACLE_MSG_SIZE, S2_OBST_MSGS);
	if (mbf_obstacle < E_OK) {
		return mbf_obstacle;
	}
	{
		T_CFLG cflg = {0};

		cflg.flgatr  = TA_TFIFO | TA_WMUL;	/* many setters, S2 reads */
		cflg.iflgptn = 0U;
		flg_faults = tk_cre_flg(&cflg);
	}
	return (flg_faults < E_OK) ? flg_faults : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	ID tid;

	/* Core 1 with the line task: the 5 ms chain stays on one core. */
	tid = app_start_task(motor_task, S2_PRI_MOTOR, TP_PRC2, "motor_task");
	if (tid < E_OK) {
		return tid;
	}
	tid = app_start_task(mission_task, S2_PRI_MISSION, TP_PRC2, "mission_task");
	return (tid < E_OK) ? tid : E_OK;
}

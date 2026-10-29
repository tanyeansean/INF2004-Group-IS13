/*
 *	subsystem5.c
 *	Subsystem 5 -- Obstacle Avoid: creates its objects and starts its task.
 *
 *	subsystem5_init():  kernel objects only (called before ANY task starts)
 *	subsystem5_start(): tasks
 */

#include "subsystem5.h"
#include "drv_hcsr04.h"
#include "s5_hw.h"

EXPORT ID	flg_avoid;		/* S2 -> S5, owned by S5 (subsystem5.h) */

#if S5_STANDALONE
EXPORT ID	s5_stub_hb_flg;		/* stand-ins until S1/S2 exist (s5_links.h) */
EXPORT ID	s5_stub_fault_flg;
EXPORT ID	s5_stub_obstacle_mbf;

/* Per-message overhead in a message buffer is a small length header;
   8 bytes per message is a safe margin. */
#define S5_STUB_MBF_BYTES	((OBSTACLE_MSG_SIZE + 8U) * S5_OBSTACLE_MBF_MSGS)

LOCAL ER s5_create_stubs(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	s5_stub_hb_flg = tk_cre_flg(&cflg);
	if (s5_stub_hb_flg < E_OK) {
		return s5_stub_hb_flg;
	}
	s5_stub_fault_flg = tk_cre_flg(&cflg);
	if (s5_stub_fault_flg < E_OK) {
		return s5_stub_fault_flg;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)S5_STUB_MBF_BYTES;
	cmbf.maxmsz = (INT)OBSTACLE_MSG_SIZE;
	s5_stub_obstacle_mbf = tk_cre_mbf(&cmbf);
	return (s5_stub_obstacle_mbf < E_OK) ? s5_stub_obstacle_mbf : E_OK;
}
#endif /* S5_STANDALONE */

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};
	ER	er;

	/* TIMER + PWM out of reset, 1 MHz tick on (see s5_hw.h). */
	s5_hw_init();

	cflg.flgatr  = TA_TFIFO | TA_WSGL;	/* only obstacle_task waits */
	cflg.iflgptn = 0U;
	flg_avoid = tk_cre_flg(&cflg);
	if (flg_avoid < E_OK) {
		return flg_avoid;
	}

	er = drv_hcsr04_create();
	if (er < E_OK) {
		return er;
	}

#if S5_STANDALONE
	er = s5_create_stubs();
#endif
	return er;
}

EXPORT ER subsystem5_start(void)
{
	ID tid;

	/* Core 0 (TP_PRC1): the echo interrupt is enabled from this task, so
	   ISR and task share one core and never run at the same instant. */
	tid = app_start_task(obstacle_task, S5_PRI_OBSTACLE, TP_PRC1, "obstacle_task");
	if (tid < E_OK) {
		return tid;
	}

#if S5_STANDALONE
	tid = app_start_task(s5_bench_task, S5_PRI_BENCH, 0, "s5_bench_task");
#endif
	return (tid < E_OK) ? tid : E_OK;
}

/*
 *	subsystem3.c
 *	Subsystem 3 -- Line Follow + Barcode Decode: objects, tasks, helpers.
 *
 *	subsystem3_init():  kernel objects only (called before ANY task starts)
 *	subsystem3_start(): tasks
 */

#include "subsystem3.h"
#include "drv_track.h"

EXPORT ID	flg_s3;		/* IF-03, owned by S3 (the receiver) */

#if TM_CONSOLE_USB_CDC
IMPORT UW	tm_usb_state(void);	/* 3 = host has the port open */
#endif

EXPORT UW s3_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return (UW)t.lo;
}

/* Wait up to 10 s for a serial terminal (like integrated_car), so the
   calibration prompts are not lost.  Safe to call from several tasks. */
EXPORT void s3_wait_console(void)
{
#if TM_CONSOLE_USB_CDC
	INT i;

	for (i = 0; (i < 100) && (tm_usb_state() < 3U); i++) {
		tk_dly_tsk(100);
	}
#endif
}

EXPORT BOOL s3_is_calibrated(void)
{
	return (s3_calibrated != 0U) ? TRUE : FALSE;
}

EXPORT void s3_request_line_reset(void)
{
	(void)tk_set_flg(flg_s3, S3_FLG_LINE_RESET);
}

EXPORT BOOL s3_line_reset_pending(void)
{
	T_RFLG	rflg;

	if (tk_ref_flg(flg_s3, &rflg) != E_OK) {
		return FALSE;
	}
	return ((rflg.flgptn & S3_FLG_LINE_RESET) != 0U) ? TRUE : FALSE;
}

EXPORT ER subsystem3_init(void)
{
	T_CFLG	cflg = {0};

	drv_track_timer_init();		/* 1 us timestamps for the barcode */
	barcode_eq_init(&s3_edge_q);

	cflg.flgatr  = TA_TFIFO | TA_WSGL;	/* only the line task waits */
	cflg.iflgptn = 0U;
	flg_s3 = tk_cre_flg(&cflg);
	return (flg_s3 < E_OK) ? flg_s3 : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	ID tid;

	/* Line follow on core 1, away from the WiFi/USB service tasks. */
	tid = app_start_task(line_task, S3_PRI_LINE, TP_PRC2, "line_task");
	if (tid < E_OK) {
		return tid;
	}
	/* Barcode on core 0: it enables the GP7 interrupt there itself. */
	tid = app_start_task(barcode_task, S3_PRI_BARCODE, TP_PRC1, "barcode_task");
	return (tid < E_OK) ? tid : E_OK;
}

/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Follow + Barcode Decode (Buddy 3, Wong Yong Quan).
 *	Student ID: 2500377.
 *
 *	Public interface of subsystem 3.  The logic is Buddy 3's track_purist
 *	(line_follow.c, ir_calibrate.c, barcode_decode.c, nav_cmd.c), unchanged
 *	from the host-tested and bare-metal-tested version; this folder holds
 *	only the uT-Kernel tasks and the hardware driver.
 *
 *	Outputs (both queues are owned by the receiver, subsystem 2):
 *	  IF-01  line_err_q  every 5 ms, line_err_t (2 bytes, track_types.h)
 *	  IF-02  barcode_q   one barcode_t (1 byte) per VALID or INVALID read
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "track_types.h"

#define S3_PRI_LINE		12	/* band 12-13 (TEAM.md): 5 ms sensing */
#define S3_PRI_BARCODE		13

/* 1 = print one line per barcode result with all element widths.
   On while the uT-Kernel barcode path is being brought up on the car. */
#ifndef BARCODE_DEBUG
#define BARCODE_DEBUG		1
#endif

/* Barcode edge-source counters for the status line: src 'I' = GPIO
   interrupt, 'S' = 0.5 ms sampler; GP7 interrupts taken; level changes
   seen by the sampler; edges given to the decoder. */
IMPORT void	s3_barcode_diag(UB *p_src, UW *p_isr, UW *p_samp, UW *p_pushed);

/* 1 once the front sensors are calibrated (READY).  Written only by the
   line task, one aligned word: safe to read from any task or core. */
IMPORT BOOL	s3_is_calibrated(void);

/* IF-03 (S2 -> S3): ask the line task to reset its junction / last-side
   state, e.g. on START.  While pending, line_err_q messages were made with
   the old state and should be ignored. */
IMPORT void	s3_request_line_reset(void);
IMPORT BOOL	s3_line_reset_pending(void);

/* Private to subsystem 3 */
IMPORT ID	flg_s3;
#define S3_FLG_LINE_RESET	(1U << 0)
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	barcode_task(INT stacd, void *exinf);
IMPORT UW	s3_now_ms(void);
IMPORT void	s3_wait_console(void);
IMPORT volatile UW	s3_calibrated;

#endif /* SUBSYSTEM3_H */

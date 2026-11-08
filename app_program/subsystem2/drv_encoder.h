/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoders (Buddy 2, Tan Yean Sean).
 *
 *	Two Hall quadrature encoders (A/B phase), PH2.0-6P motor connector:
 *	  01 GND, 02 output A, 03 output B, 04 power (3V3 from the Grove port)
 *	  Left  wheel: Grove 4, A = GP16, B = GP17
 *	  Right wheel: Grove 1, A = GP0,  B = GP1
 *
 *	Counting: every edge of A and B (x4 decoding), signed 32-bit, never
 *	reset.  Users keep their own previous value and take differences.
 *	Positive = wheel turning forward (fix with ENC_INVERT_* if not).
 *
 *	Interrupt: the edges are served by the ONE shared IO_IRQ_BANK0 handler
 *	in subsystem3/drv_track.c, which calls drv_encoder_irq().  That
 *	interrupt runs on core 0 only, so drv_encoder_init() enables the four
 *	pins in core 0's PROC0_INTE whatever core calls it.
 *
 *	GP0/GP1 are UART0 in "make CRASH_REPORT=1" / "CONSOLE=uart" builds: the
 *	right encoder is then left switched off and reads 0.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "../app.h"

#define ENC_PIN_LEFT_A		16U
#define ENC_PIN_LEFT_B		17U
#define ENC_PIN_RIGHT_A		0U
#define ENC_PIN_RIGHT_B		1U

/* 1 = the wheel counts down when it turns forward: flip it.  Set from
   test S2-U03 / S2-U04 (make TEST=1).  Left measured on the car: counts
   DOWN going forward (motors are mirrored), right counts up. */
#ifndef ENC_INVERT_LEFT
#define ENC_INVERT_LEFT		1
#endif
#ifndef ENC_INVERT_RIGHT
#define ENC_INVERT_RIGHT	0
#endif

/* Wheel incl. rubber tyre, measured 6.5 cm.  Counts per wheel revolution
   is not known yet: measure it with S2-U03 / S2-U04. */
#define ENC_WHEEL_DIAM_MM	65U

#if BOARD_UART0_FREE
#define ENC_RIGHT_ENABLED	1
#else
#define ENC_RIGHT_ENABLED	0	/* GP0/GP1 belong to UART0 in this build */
#endif

IMPORT ER	drv_encoder_init(void);
IMPORT void	drv_encoder_irq(UW core);	/* shared GPIO handler only */
IMPORT void	drv_encoder_read(W *p_left, W *p_right);
IMPORT void	drv_encoder_levels(UW *p_ab_l, UW *p_ab_r);	/* (A<<1)|B */
IMPORT void	drv_encoder_diag(UW *p_edges_l, UW *p_edges_r,
				 UW *p_bad_l, UW *p_bad_r);

#endif /* DRV_ENCODER_H */

/*
 *	drv_track.h
 *	Subsystem 3 -- IR sensor hardware.  Private to subsystem 3.
 */

#ifndef DRV_TRACK_H
#define DRV_TRACK_H

#include "../app.h"
#include "barcode_decode.h"

#define S3_PIN_LEFT_ADC		(27U)	/* Grove 6, left front IR, ADC1  */
#define S3_PIN_RIGHT_ADC	(26U)	/* Grove 5, right front IR, ADC0 */
#define S3_ADC_CH_LEFT		(1)
#define S3_ADC_CH_RIGHT		(0)
#define S3_PIN_BARCODE		(7U)	/* Grove 7, barcode IR D0        */

IMPORT barcode_edge_queue_t	s3_edge_q;

IMPORT void	drv_track_timer_init(void);
IMPORT UW	drv_track_now_us(void);

/* Line task only. */
IMPORT ER	drv_track_adc_open(void);
IMPORT UH	drv_track_read_left(void);
IMPORT UH	drv_track_read_right(void);

/* Barcode task only (call it from that task: the ISRs then run on its core).
   Sets up the GPIO edge interrupt AND the 0.5 ms sampler; the interrupt
   feeds the queue unless drv_track_barcode_use_sampler() was called. */
IMPORT ER	drv_track_barcode_irq_init(void);
IMPORT void	drv_track_barcode_use_sampler(void);
IMPORT UW	drv_track_barcode_level(void);

/* Edge-source diagnostics (written by the ISRs, read anywhere). */
#define S3_BAR_SRC_IRQ		(0U)
#define S3_BAR_SRC_SAMPLER	(1U)
IMPORT volatile UW	s3_bar_src;
IMPORT volatile UW	s3_bar_isr_count;
IMPORT volatile UW	s3_bar_samp_changes;
IMPORT volatile UW	s3_bar_pushed;
IMPORT ER		s3_bar_irq_er;
IMPORT ER		s3_bar_samp_er;

#endif /* DRV_TRACK_H */

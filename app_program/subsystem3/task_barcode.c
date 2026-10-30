/*
 *	task_barcode.c
 *	Subsystem 3 -- Barcode Decode task.  Drains the edge timestamps the
 *	GP7 interrupt queued, runs the size/speed-independent Code 39
 *	assembler (barcode_decode.c) and sends every result, VALID or
 *	INVALID, to Mission Control (barcode_q, zero-timeout send).
 *
 *	Same result handling as integrated_car/main.c on_barcode_result():
 *	dropped edges force INVALID; INVALID reads are printed here (only this
 *	task knows the edge counts); Mission Control prints VALID ones with
 *	the map lookup.  Pinned to core 0 together with the interrupt.
 */

#include "subsystem3.h"
#include "drv_track.h"
#include "barcode_decode.h"
#include "../subsystem1/subsystem1.h"	/* flg_hb, HB_BARCODE_DECODE */
#include "../subsystem2/subsystem2.h"	/* IF-02: mbf_barcode */

#define S3_BARCODE_POLL_MS	5	/* well under the 20 ms the decoder needs */

LOCAL UW	s3_barcode_dropped = 0U;	/* barcode_q full */

LOCAL void on_result(const barcode_asm_t *p_asm, barcode_t res, UB *p_seen_dropped)
{
	barcode_frame_info_t	info;

	barcode_asm_last_frame(p_asm, &info);
	if (barcode_eq_overflow_delta(&s3_edge_q, p_seen_dropped) != 0U) {
		res = barcode_invalid();	/* edges were dropped: untrusted */
	}

	if (!barcode_is_valid(res)) {
		if (info.outcome == BARCODE_OUT_PARTIAL) {
			tm_printf((UB *)"[bar] INVALID (partial: %u of 30 edges)\n",
				  (UINT)info.n_edges);
		} else {
			tm_printf((UB *)"[bar] INVALID (%u edges, no window decoded)\n",
				  (UINT)info.burst_edges);
		}
	}

#if BARCODE_DEBUG
	{
		UB i;
		tm_printf((UB *)"[bar dbg] %s edges=%u burst=%u chatter=%u longest=%u.%u ms widths ms:",
			  barcode_is_valid(res) ? "VALID" : "INVALID",
			  (UINT)info.n_edges, (UINT)info.burst_edges, (UINT)info.chatter_pairs,
			  info.longest_us / 1000U, (info.longest_us % 1000U) / 100U);
		for (i = 0U; i < info.n_widths; i++) {
			tm_printf((UB *)" %u.%u", info.widths_us[i] / 1000U,
				  (info.widths_us[i] % 1000U) / 100U);
		}
		tm_printf((UB *)"\n");
	}
#endif

	if (tk_snd_mbf(mbf_barcode, &res, (INT)sizeof(res), TMO_POL) != E_OK) {
		s3_barcode_dropped++;
	}
}

EXPORT void s3_barcode_diag(UB *p_src, UW *p_isr, UW *p_samp, UW *p_pushed)
{
	*p_src    = (s3_bar_src == S3_BAR_SRC_SAMPLER) ? (UB)'S' : (UB)'I';
	*p_isr    = s3_bar_isr_count;
	*p_samp   = s3_bar_samp_changes;
	*p_pushed = s3_bar_pushed;
}

/* GP7 interrupt never fired although the sampler saw the line change:
   switch to the sampler (one-way, reported once). */
#define S3_SWITCH_MIN_CHANGES	(6U)

LOCAL void check_edge_source(void)
{
	static BOOL	reported = FALSE;

	if ((s3_bar_src == S3_BAR_SRC_IRQ) && (s3_bar_isr_count == 0U) &&
	    (s3_bar_samp_changes >= S3_SWITCH_MIN_CHANGES) && (s3_bar_samp_er >= E_OK)) {
		drv_track_barcode_use_sampler();
		tm_printf((UB *)"[bar] GP7 interrupt did not fire (sampler saw %u changes)"
			  " - now using the 0.5 ms sampler\n", (UINT)s3_bar_samp_changes);
	}
	if (!reported && s3_is_calibrated()) {
		reported = TRUE;	/* console is open by now */
		tm_printf((UB *)"[bar] edge source: %s (gpio irq=%d, sampler=%d), GP7=%u\n",
			  (s3_bar_src == S3_BAR_SRC_SAMPLER) ? "0.5 ms sampler" : "GPIO interrupt",
			  (INT)s3_bar_irq_er, (INT)s3_bar_samp_er,
			  (UINT)drv_track_barcode_level());
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	barcode_asm_t	basm;
	barcode_edge_t	e;
	barcode_t	res;
	UB		seen_dropped = 0U;
	ER		er;

	(void)stacd; (void)exinf;

	barcode_asm_init(&basm);
	(void)barcode_eq_overflow_delta(&s3_edge_q, &seen_dropped);

	/* From this task, so the interrupts are enabled on this task's core. */
	er = drv_track_barcode_irq_init();
	CHECK(er, "barcode GPIO interrupt");

	for (;;) {
		check_edge_source();
		while (barcode_eq_pop(&s3_edge_q, &e)) {
			if (barcode_asm_edge_arrived(&basm, e.t_us, &res)) {
				on_result(&basm, res, &seen_dropped);
			}
		}
		/* accepts a pending edge after 2 ms, or abandons a quiet frame */
		if (barcode_asm_time_passed(&basm, drv_track_now_us(), &res)) {
			on_result(&basm, res, &seen_dropped);
		}
		(void)tk_set_flg(flg_hb, HB_BARCODE_DECODE);
		tk_dly_tsk(S3_BARCODE_POLL_MS);
	}
}

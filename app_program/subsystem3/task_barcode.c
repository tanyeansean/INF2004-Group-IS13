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

/* ---- Per-pass summary ---------------------------------------------------
 * on_result() can only report a frame the assembler actually holds, so a
 * pass whose edges never reach the decoder is silent by construction.
 * These counters live in the driver, before the decoder, so every pass
 * that moved GP7 at all becomes visible - including the ones that produced
 * no result.  One line per pass, printed when the burst goes quiet.
 */
#define S3_BAR_PASS_QUIET_US	150000U	/* == BARCODE_ABORT_FLOOR_US */
/* ...but the decoder's own abort timeout grows with the longest element it
 * has seen (up to 2 s), so a trickle of edges 200-400 ms apart keeps one
 * frame open forever.  Report anyway once queued edges have been silent
 * past that cap: the result, if any, has already been produced by then. */
#define S3_BAR_PASS_STALL_US	2500000U
/* A pass that produced queued edges keeps being extended only by NEW queued
 * edges, never by raw GP7 chatter: a comparator that oscillates at kHz would
 * otherwise keep the pass "active" forever and the report would never print.
 * This cap is the safety valve for a pass with a slow trickle of edges. */
#define S3_BAR_PASS_MAX_US	5000000U

LOCAL barcode_t	s3_pass_last_res;	/* last result produced in this pass */
LOCAL BOOL	s3_pass_have_res;	/* FALSE = decoder rejected every edge */
LOCAL UW	s3_pass_raw;		/* GP7 changes, before the sampler debounce */
LOCAL UW	s3_pass_queued;		/* edges actually queued for the decoder   */
LOCAL UW	s3_pass_dropped;	/* edge-queue overflows in this pass       */
LOCAL UW	s3_pass_start_us;	/* when this pass began                    */
LOCAL UW	s3_pass_last_push_us;	/* newest QUEUED edge in this pass         */
LOCAL UW	s3_last_raw_us;		/* newest GP7 change, for idle detection   */
LOCAL BOOL	s3_pass_active;

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
	s3_pass_last_res = res;
	s3_pass_have_res = TRUE;
}

/* One line per pass: what the sensor did, what reached the decoder, and
   what came out.  Printed from the driver counters, so it still appears
   when the decoder had nothing to say. */
LOCAL void bar_pass_report(void)
{
	if (s3_pass_have_res && barcode_is_valid(s3_pass_last_res)) {
		tm_printf((UB *)"[bar] pass: %u of %u edges (GP7 %u, queued %u,"
			  " dropped %u) -> '%c'\n",
			  (UINT)s3_pass_queued, (UINT)BARCODE_FRAME_EDGES,
			  (UINT)s3_pass_raw, (UINT)s3_pass_queued,
			  (UINT)s3_pass_dropped,
			  barcode_get_char(s3_pass_last_res));
	} else if (s3_pass_have_res) {
		tm_printf((UB *)"[bar] pass: %u of %u edges (GP7 %u, queued %u,"
			  " dropped %u) -> INVALID\n",
			  (UINT)s3_pass_queued, (UINT)BARCODE_FRAME_EDGES,
			  (UINT)s3_pass_raw, (UINT)s3_pass_queued,
			  (UINT)s3_pass_dropped);
	} else {
		tm_printf((UB *)"[bar] pass: %u of %u edges (GP7 %u, queued %u,"
			  " dropped %u) -> no result (all edges rejected)\n",
			  (UINT)s3_pass_queued, (UINT)BARCODE_FRAME_EDGES,
			  (UINT)s3_pass_raw, (UINT)s3_pass_queued,
			  (UINT)s3_pass_dropped);
	}
}

EXPORT void s3_barcode_diag(UB *p_src, UW *p_isr, UW *p_samp, UW *p_pushed,
			     UW *p_raw)
{
	*p_src    = (s3_bar_src == S3_BAR_SRC_SAMPLER) ? (UB)'S' : (UB)'I';
	*p_isr    = s3_bar_isr_count;
	*p_samp   = s3_bar_samp_changes;
	*p_pushed = s3_bar_pushed;
	*p_raw    = s3_bar_raw_changes;
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
	UW		prev_raw, prev_pushed, prev_dropped;
	UW		raw, pushed, dropped, now_us;

	(void)stacd; (void)exinf;

	barcode_asm_init(&basm);
	(void)barcode_eq_overflow_delta(&s3_edge_q, &seen_dropped);

	/* From this task, so the interrupts are enabled on this task's core. */
	er = drv_track_barcode_irq_init();
	CHECK(er, "barcode GPIO interrupt");

	prev_raw       = s3_bar_raw_changes;
	prev_pushed    = s3_bar_pushed;
	prev_dropped   = s3_edge_q.dropped;
	s3_pass_active = FALSE;

	for (;;) {
		now_us = drv_track_now_us();
		check_edge_source();
		while (barcode_eq_pop(&s3_edge_q, &e)) {
			if (barcode_asm_edge_arrived(&basm, e.t_us, &res)) {
				on_result(&basm, res, &seen_dropped);
			}
		}
		/* accepts a pending edge after 2 ms, or abandons a quiet frame */
		if (barcode_asm_time_passed(&basm, now_us, &res)) {
			on_result(&basm, res, &seen_dropped);
		}

		/* ---- per-pass tracker ---------------------------------------
		   Reads the driver counters, not the decoder, so a pass whose
		   edges never got that far is still reported.  A pass STARTS on
		   any GP7 activity, but is only EXTENDED by a newly queued edge:
		   raw chatter alone must not postpone the report.  After a pass
		   is reported, a new one starts on the next queued edge, or once
		   GP7 has been quiet long enough that the chatter is over. */
		raw     = s3_bar_raw_changes;
		pushed  = s3_bar_pushed;
		dropped = s3_edge_q.dropped;

		if (raw != prev_raw) {
			s3_last_raw_us = now_us;
		}
		if (!s3_pass_active) {
			BOOL const raw_quiet =
				(UW)(now_us - s3_last_raw_us) > S3_BAR_PASS_QUIET_US;

			if ((pushed != prev_pushed) ||
			    ((raw != prev_raw) && raw_quiet)) {
				s3_pass_active      = TRUE;
				s3_pass_have_res    = FALSE;
				s3_pass_raw         = 0U;
				s3_pass_queued      = 0U;
				s3_pass_dropped     = 0U;
				s3_pass_start_us    = now_us;
				s3_pass_last_push_us = now_us;
			}
		}
		if (s3_pass_active) {
			s3_pass_raw    += raw - prev_raw;
			s3_pass_queued += pushed - prev_pushed;
			if (pushed != prev_pushed) {
				s3_pass_last_push_us = now_us;
			}
			if (dropped != prev_dropped) {
				s3_pass_dropped += (UW)(UB)(dropped - prev_dropped);
			}
		}
		prev_raw     = raw;
		prev_pushed  = pushed;
		prev_dropped = dropped;

		/* The pass is over when the QUEUED edges have been quiet past the
		   abort floor and the assembler holds nothing, when they have been
		   silent longer than the assembler's own 2 s abort cap (a trickle
		   of noise edges must not hold the report hostage), or when the
		   pass has simply run too long.  One line per pass either way. */
		if (s3_pass_active) {
			BOOL const asm_idle = (basm.count == 0U) && (basm.have_pending == 0U);
			BOOL const quiet    = (UW)(now_us - s3_pass_last_push_us) > S3_BAR_PASS_QUIET_US;
			BOOL const stalled  = (UW)(now_us - s3_pass_last_push_us) > S3_BAR_PASS_STALL_US;
			BOOL const too_long = (UW)(now_us - s3_pass_start_us) > S3_BAR_PASS_MAX_US;

			if ((quiet && asm_idle) || stalled || too_long) {
				s3_pass_active = FALSE;
				bar_pass_report();
			}
		}

		(void)tk_set_flg(flg_hb, HB_BARCODE_DECODE);
		tk_dly_tsk(S3_BARCODE_POLL_MS);
	}
}

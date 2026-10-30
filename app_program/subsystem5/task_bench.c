/*
 *	task_bench.c
 *	Subsystem 5 -- BENCH ONLY (S5_STANDALONE=1): a stand-in for Mission
 *	Control and the Safety Monitor, so S5 can be tested on its own.
 *
 *	  - prints every obstacle_q message, decoded
 *	  - answers like Mission Control would:
 *	      OBSTACLE         -> MOTORS_STOPPED after 100 ms
 *	      PLAN LEFT/RIGHT  -> LINE_REACQUIRED after 1 s
 *	      PLAN REVERSE     -> MOTORS_STOPPED after 500 ms
 *	      FAULT            -> RESUME after 3 s
 *	  - checks S5's heartbeat like the Safety Monitor: prints
 *	    "HEARTBEAT MISSED" if the bit is not seen for > 120 ms.
 *	Delete or disable (S5_STANDALONE=0) once S1 and S2 are integrated.
 */

#include "subsystem5.h"

#if S5_STANDALONE

#define BENCH_POLL_MS		(20)
#define BENCH_HB_LIMIT_MS	(120U)

LOCAL const char * const k_type_name[] = { "OBSTACLE", "CLEAR", "PLAN", "FAULT" };
LOCAL const char * const k_plan_name[] = { "STOP", "CONTINUE", "LEFT", "RIGHT",
					   "REVERSE", "?", "?", "?" };

LOCAL UW now_ms(void)
{
	SYSTIM t;

	(void)tk_get_tim(&t);
	return t.lo;
}

EXPORT void s5_bench_task(INT stacd, void *exinf)
{
	uint8_t		buf[OBSTACLE_MSG_SIZE];
	obstacle_msg_t	msg;
	INT		len;
	UINT		flgptn;
	UW		reply_bits  = 0U;
	UW		reply_at_ms = 0U;
	UW		last_hb_ms;
	UW		worst_gap_ms = 0U;
	BOOL		b_valid;

	(void)stacd;
	(void)exinf;

	tm_printf((UB*)"[S5 bench] stand-in Mission Control running\n");
	(void)tk_set_flg(flg_avoid, AVOID_EVT_MC_READY);	/* obstacle_task waits for it */
	last_hb_ms = now_ms();

	for (;;) {
		len = tk_rcv_mbf(S5_OBSTACLE_MBF, buf, BENCH_POLL_MS);
		if (len == (INT)OBSTACLE_MSG_SIZE) {
			b_valid = obstacle_msg_unpack(buf, &msg) ? TRUE : FALSE;
			tm_printf((UB*)"[S5 bench] %s plan=%s state=%d closest=%d width=%d "
				  "L=%d R=%d side=%d%s\n",
				  k_type_name[msg.type & 3U], k_plan_name[msg.plan & 7U],
				  msg.state, msg.closest_cm, msg.width_cm,
				  msg.left_clear_cm, msg.right_clear_cm, msg.sidestep_cm,
				  b_valid ? "" : "  (INVALID)");

			switch (msg.type) {
			case OBS_MSG_OBSTACLE:
				reply_bits = AVOID_EVT_MOTORS_STOPPED; reply_at_ms = now_ms() + 100U;
				break;
			case OBS_MSG_PLAN:
				if ((msg.plan == OBS_PLAN_LEFT) || (msg.plan == OBS_PLAN_RIGHT)) {
					reply_bits = AVOID_EVT_LINE_REACQUIRED; reply_at_ms = now_ms() + 1000U;
				} else if (msg.plan == OBS_PLAN_REVERSE) {
					reply_bits = AVOID_EVT_MOTORS_STOPPED; reply_at_ms = now_ms() + 500U;
				} else {
					/* STOP: FAULT follows. */
				}
				break;
			case OBS_MSG_FAULT:
				reply_bits = AVOID_EVT_RESUME; reply_at_ms = now_ms() + 3000U;
				break;
			default:
				break;
			}
		}

		if ((reply_bits != 0U) && ((INT)(now_ms() - reply_at_ms) >= 0)) {
			(void)tk_set_flg(flg_avoid, (UINT)reply_bits);
			reply_bits = 0U;
		}

		/* Safety-Monitor-style heartbeat check: read and clear S5's bit. */
		if (tk_wai_flg(S5_HB_FLG, S5_HB_BIT, TWF_ORW | TWF_BITCLR,
			       &flgptn, TMO_POL) == E_OK) {
			UW const gap = now_ms() - last_hb_ms;

			if (gap > worst_gap_ms) {
				worst_gap_ms = gap;
				tm_printf((UB*)"[S5 bench] worst heartbeat gap so far %d ms\n",
					  (INT)worst_gap_ms);
			}
			last_hb_ms = now_ms();
		} else if ((now_ms() - last_hb_ms) > BENCH_HB_LIMIT_MS) {
			tm_printf((UB*)"[S5 bench] HEARTBEAT MISSED (%d ms)\n",
				  (INT)(now_ms() - last_hb_ms));
			last_hb_ms = now_ms();
		} else {
			/* Waiting. */
		}
	}
}

#endif /* S5_STANDALONE */

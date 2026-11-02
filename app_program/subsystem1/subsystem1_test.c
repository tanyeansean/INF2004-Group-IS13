/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.  IDs: S1-U01, ...
 *	(The packet layouts have host tests in robocar_comms test/run_tests.py.)
 */

#include "subsystem1.h"
#include "packets.h"
#include "safety_logic.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	uint8_t	buf[PKT_LIVE_LEN] = {0};

	TEST_INIT();

	/* S1-U01: packed field round trip on the target compiler */
	pkt_put(buf, PKT_LIVE_LINE_POS, (uint32_t)(-37 & 0x7F));
	TEST_CHECK("S1-U01", pkt_get_signed(buf, PKT_LIVE_LINE_POS) == -37,
		   "7-bit signed line position packs and unpacks");

	/* S1-U02: the Telemetry task is alive (sets its heartbeat bit).  Only
	   waits: the Safety Monitor is the one task that clears flg_hb. */
	{
		UINT	ptn = 0U;
		ER	er;

		er = tk_wai_flg(flg_hb, HB_TELEMETRY, TWF_ORW, &ptn, 100);
		TEST_CHECK("S1-U02", er == E_OK, "telemetry heartbeat within 100 ms");
	}

	/* S1-U03..U07: Safety Monitor attendance logic (safety_logic.c) */
	{
		sm_state_t	sm;
		UW		rec = 0U;

		sm_init(&sm);
		(void)sm_check(&sm, 0U, 1000U, &rec);
		TEST_CHECK("S1-U03", sm.hung == 0U, "a task that never beat is not reported hung");
		sm_seen(&sm, HB_MOTOR_CONTROL | HB_OBSTACLE_AVOID, 1000U);
		TEST_CHECK("S1-U04", sm_check(&sm, 0U, 1000U + SM_LIMIT_MS, &rec) == 0U,
			   "no beat for exactly the limit is still on time");
		TEST_CHECK("S1-U05", sm_check(&sm, 0U, 1010U + SM_LIMIT_MS, &rec) == HB_MOTOR_CONTROL,
			   "Motor Control hung after its limit; Obstacle Avoid has longer");
		(void)sm_check(&sm, HB_MOTOR_CONTROL, 1100U, &rec);
		TEST_CHECK("S1-U06", (sm.hung == 0U) && (rec == HB_MOTOR_CONTROL),
			   "a beat clears hung and reports the recovery");
		TEST_CHECK("S1-U07", (sm_first(HB_LINE_FOLLOW | HB_TELEMETRY) == 3U) && (sm_first(0U) == 0U),
			   "hung task number for telemetry (Line Follow = 3)");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

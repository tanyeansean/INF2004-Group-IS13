/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.  IDs: S1-U01, ...
 *	(The packet layouts have host tests in robocar_comms test/run_tests.py.)
 */

#include "subsystem1.h"
#include "packets.h"
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

	/* S1-U02: the Telemetry task is alive (sets its heartbeat bit) */
	{
		UINT	ptn = 0U;
		ER	er;

		(void)tk_clr_flg(flg_hb, ~HB_TELEMETRY);
		er = tk_wai_flg(flg_hb, HB_TELEMETRY, TWF_ORW, &ptn, 100);
		TEST_CHECK("S1-U02", er == E_OK, "telemetry heartbeat within 100 ms");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

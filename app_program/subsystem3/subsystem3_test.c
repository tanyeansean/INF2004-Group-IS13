/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record results in testing/subsystem3/TEST_LOG.md.  IDs: S3-U01, ...
 *	(The decoder and line logic have host tests in track_purist/test.)
 */

#include "subsystem3.h"
#include "drv_track.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	UW	t0, t1;

	TEST_INIT();

	/* S3-U01: the 1 us timer runs (needed for barcode timestamps) */
	t0 = drv_track_now_us();
	tk_dly_tsk(10);
	t1 = drv_track_now_us();
	TEST_CHECK("S3-U01", (t1 - t0) >= 9000U && (t1 - t0) <= 20000U,
		   "1 us timer advances ~10 ms in a 10 ms delay");

	/* S3-U02: the barcode line idles at a defined level (pull-up) */
	TEST_CHECK("S3-U02", drv_track_barcode_level() <= 1U, "GP7 reads 0 or 1");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

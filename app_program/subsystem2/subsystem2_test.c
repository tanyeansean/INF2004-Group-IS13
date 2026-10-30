/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	UW	m;

	TEST_INIT();

	/* S2-U01: wheel message packs both signs and clamps */
	m = s2_wheels_pack(-255, 300);
	TEST_CHECK("S2-U01", s2_wheel_get(m, 0U) == -255 &&
		   s2_wheel_get(m, S2_WHEEL_BITS) == 255 && (m >> 18) == 0U,
		   "wheel PWM pair packs into 18 bits, clamped to +/-255");

	/* S2-U02: the stored map uses only bits 0-20 (20 map bits + valid) */
	TEST_CHECK("S2-U02", (nav_map_snapshot(&g_nav_map) >> 21) == 0U,
		   "barcode map snapshot fits 21 bits");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

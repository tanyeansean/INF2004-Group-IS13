/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01, ...
 *
 *	Run at boot from the initial task, with the car FLAT and STILL.  The
 *	terrain task is waiting for line calibration at that point and does
 *	not use the sensor, so these reads don't collide with it.
 *
 *	  S4-U01  the GY-511 answers a read
 *	  S4-U02  a still car reads about 1 g in total (0.7 to 1.3 g)
 *	  S4-U03  ten reads in a row all succeed
 *	  S4-U04  the report word unpacks to the values packed into it
 *	  S4-U05  detector on the Pico: 1 s of flat readings gives no hump and
 *	          no lift
 *	  S4-U06  detector on the Pico: turned upside down gives OVERTURNED
 *	  S4-U07  s4_start_ok() says "no" while the IMU has not calibrated yet
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "terrain_logic.h"
#include "../test_report.h"

#if APP_TEST

#define S4_TEST_SETTLE_MS	200	/* let terrain_task finish starting the sensor */
#define S4_TEST_READS		10
#define S4_TEST_STEPS		100	/* 100 readings = 1 s */

EXPORT void subsystem4_test(void)
{
	H		ax = 0, ay = 0, az = 0;
	ER		er;
	float		x, y, z, total2;
	INT		i, good = 0;
	UW		r;
	terrain_t	t;
	UINT		ev = 0U;

	TEST_INIT();

	tk_dly_tsk(S4_TEST_SETTLE_MS);

	/* ---- the sensor ---- */
	er = drv_imu_read(&ax, &ay, &az);
	TEST_CHECK("S4-U01", er >= E_OK, "GY-511 answers a read");

	x = ax / IMU_LSB_PER_G;
	y = ay / IMU_LSB_PER_G;
	z = az / IMU_LSB_PER_G;
	total2 = (x * x) + (y * y) + (z * z);
	TEST_CHECK("S4-U02", (total2 > 0.49f) && (total2 < 1.69f),
		   "still car reads about 1 g in total");

	for (i = 0; i < S4_TEST_READS; i++) {
		if (drv_imu_read(&ax, &ay, &az) >= E_OK) {
			good++;
		}
		tk_dly_tsk(10);
	}
	TEST_CHECK("S4-U03", good == S4_TEST_READS, "10 reads in a row succeed");

	/* ---- the report word (layout in subsystem4.h) ----
	   tilt 12.5 deg (25), turn -40 deg/s (-20), motion 3, terrain 1,
	   lifted 2, last hump 10 mm, hump counter 5 */
	r = (25U) | (((UW)(-20) & 0xFFU) << 7) | (3U << 15) | (1U << 18) |
	    (2U << 20) | (10U << 22) | (5U << 29);
	TEST_CHECK("S4-U04",
		   (s4_report_tilt_deg(r) == 12.5f) && (s4_report_turn_dps(r) == -40) &&
		   (s4_report_motion(r) == 3U) && (s4_report_terrain(r) == 1U) &&
		   (s4_report_lift(r) == 2U) && (s4_report_last_hump_mm(r) == 10U) &&
		   (s4_report_humps(r) == 5U),
		   "report word unpacks to what was packed");

	/* ---- the detector, run on the Pico itself (same code as the laptop
	   test: this checks the Pico's own floating-point maths) ---- */
	terrain_init(&t, 0.0f, 0.0f, -1.0f);		/* flat: Z = -1 g */
	for (i = 0; i < S4_TEST_STEPS; i++) {
		ev |= terrain_step(&t, 0.0f, 0.0f, -1.0f);
	}
	TEST_CHECK("S4-U05", (ev == 0U) && (t.humps == 0) && (t.lift == TERRAIN_ON_FLOOR),
		   "flat for 1 s: no hump, no lift");

	ev = 0U;
	for (i = 0; i < S4_TEST_STEPS; i++) {
		ev |= terrain_step(&t, 0.0f, 0.0f, 1.0f);	/* upside down: Z = +1 g */
	}
	TEST_CHECK("S4-U06", ((ev & TERRAIN_EV_OVERTURNED) != 0U) && (t.lift == TERRAIN_OVERTURNED),
		   "upside down: OVERTURNED");

	/* ---- the START check: line calibration has not happened yet, so the
	   IMU has not calibrated and the car must not be startable ---- */
	TEST_CHECK("S4-U07", (s4_status_word != S4_ST_CALIBRATING) || !s4_start_ok(),
		   "no START while the IMU is calibrating");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

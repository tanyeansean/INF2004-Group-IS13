/*
 *	task_terrain.c
 *	Subsystem 4 -- Terrain Monitor task: calibration once the line sensors
 *	are done, then every 10 ms reads the GY-511, runs the hump and collision
 *	detector (terrain_logic.c) and prints what it finds.
 *
 *	Why a cyclic handler: the kernel's clock ticks every 10 ms
 *	(CNF_TIMER_PERIOD), and tk_dly_tsk(10) adds one tick for safety, so a
 *	delay loop would really read every 20 ms.  The detector was tested at
 *	10 ms, so the handler wakes the task once per tick instead.
 *
 *	The task never drives the motors.  It doesn't report to the rest of
 *	the car yet (no heartbeat, fault bits or terrain_q): next step.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "terrain_logic.h"
#include "../subsystem3/subsystem3.h"	/* s3_is_calibrated() */

#define READY_WAIT_MAX_S	30	/* line calibration failed: go on anyway */
#define CAL_WAIT_S		5
#define CAL_DISCARD		10	/* the first readings after start aren't good */
#define CAL_SAMPLES		20
#define CAL_GAP_MS		30
#define STATUS_EVERY		100	/* status line every 100 reads = 1 s */

LOCAL ID	cyc_terrain;
LOCAL ID	tid_terrain;

/* Print a value as "12.3" */
#define TENTHS(v)		((INT)(((v) * 10.0f) + 0.5f))
#define WHOLE(v)		(TENTHS(v) / 10)
#define TENTH(v)		(TENTHS(v) % 10)

/* Print a signed reading in g as "-0.91" */
LOCAL void print_g(const char *name, float v)
{
	INT const n = (INT)(((v < 0.0f) ? -v : v) * 100.0f + 0.5f);

	tm_printf((UB *)"%s %c%d.%02d ", name, (v < 0.0f) ? '-' : '+', n / 100, n % 100);
}

LOCAL void terrain_cyc(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_terrain);
}

/* Keep trying until the sensor answers, so a loose wire can be fixed while
   the rest of the car keeps running. */
LOCAL void sensor_start(void)
{
	while (drv_imu_init() < E_OK) {
		tm_printf((UB *)"[imu] GY-511 not found at 0x1D - check wiring\n");
		tk_dly_tsk(1000);
	}
}

/* Wait for line calibration: the car is being moved by hand until READY. */
LOCAL void wait_for_line_calibration(void)
{
	INT s;

	for (s = 0; (s < READY_WAIT_MAX_S) && !s3_is_calibrated(); s++) {
		tk_dly_tsk(1000);
	}
}

/* Average some readings with the car flat and still.  Gives the direction
   of gravity on this floor. */
LOCAL void calibrate(float *p_x, float *p_y, float *p_z)
{
	float	x = 0.0f, y = 0.0f, z = 0.0f;
	INT	good = 0;
	INT	i;
	H	ax, ay, az;

	tm_printf((UB *)"[imu] Put the car FLAT and don't touch it. Calibrating in %d s...\n",
		  CAL_WAIT_S);
	tk_dly_tsk(CAL_WAIT_S * 1000);

	for (i = 0; i < CAL_DISCARD; i++) {
		(void)drv_imu_read(&ax, &ay, &az);
		tk_dly_tsk(CAL_GAP_MS);
	}
	for (i = 0; i < CAL_SAMPLES; i++) {
		if (drv_imu_read(&ax, &ay, &az) >= E_OK) {
			x += ax / IMU_LSB_PER_G;
			y += ay / IMU_LSB_PER_G;
			z += az / IMU_LSB_PER_G;
			good++;
		}
		tk_dly_tsk(CAL_GAP_MS);
	}
	if (good > 0) {
		x /= good;
		y /= good;
		z /= good;
	}
	*p_x = x;
	*p_y = y;
	*p_z = z;

	tm_printf((UB *)"[imu] Calibrated from %d readings (", good);
	print_g("X", x);
	print_g("Y", y);
	print_g("Z", z);
	tm_printf((UB *)")\n");
}

LOCAL void print_events(UINT ev, const terrain_t *t)
{
	float const peak = t->hump_peak_deg;

	if ((ev & TERRAIN_EV_COLLISION) != 0U) {
		tm_printf((UB *)"[imu] *** COLLISION! jolt %d.%d g ***\n",
			  WHOLE(t->jolt_g), TENTH(t->jolt_g));
	}
	if ((ev & TERRAIN_EV_CLIMBING) != 0U) {
		tm_printf((UB *)"[imu] >>> CLIMBING\n");
	}
	if ((ev & TERRAIN_EV_CLIMB_AGAIN) != 0U) {
		tm_printf((UB *)"[imu] >>> CLIMBING again\n");
	}
	if ((ev & TERRAIN_EV_DESCENDING) != 0U) {
		tm_printf((UB *)"[imu] >>> DESCENDING (peak tilt %d.%d deg)\n",
			  WHOLE(peak), TENTH(peak));
	}
	if ((ev & TERRAIN_EV_HUMP_DONE) != 0U) {
		tm_printf((UB *)"[imu] >>> HUMP DONE: peak tilt %d.%d deg, height %d.%d mm "
			  "(highest this run %d.%d mm)\n",
			  WHOLE(peak), TENTH(peak),
			  WHOLE(t->height_mm), TENTH(t->height_mm),
			  WHOLE(t->best_mm), TENTH(t->best_mm));
	}
	if ((ev & TERRAIN_EV_NOT_A_HUMP) != 0U) {
		tm_printf((UB *)"[imu] >>> Not a hump (peak tilt %d.%d deg), ignored\n",
			  WHOLE(peak), TENTH(peak));
	}
	if ((ev & TERRAIN_EV_TIMED_OUT) != 0U) {
		tm_printf((UB *)"[imu] >>> Timed out, treating this as flat\n");
	}
}

EXPORT void terrain_task(INT stacd, void *exinf)
{
	LOCAL const char * const state_name[] = { "FLAT", "CLIMBING", "DESCENDING" };
	terrain_t	t;
	float		x, y, z;
	float		peak_jolt = 0.0f;	/* biggest jolt since the last print */
	UW		reads = 0U;		/* good reads since the last print */
	UW		fails = 0U;		/* failed reads since the last print */
	INT		count = 0;

	(void)stacd; (void)exinf;

	sensor_start();
	wait_for_line_calibration();
	calibrate(&x, &y, &z);
	terrain_init(&t, x, y, z);

	/* From here on, one pass per wake-up: every 10 ms */
	tid_terrain = tk_get_tid();
	CHECK(tk_sta_cyc(cyc_terrain), "terrain cyc");

	for (;;) {
		H	ax, ay, az;
		UINT	ev;

		(void)tk_slp_tsk(TMO_FEVR);

		if (drv_imu_read(&ax, &ay, &az) >= E_OK) {
			ev = terrain_step(&t, ax / IMU_LSB_PER_G, ay / IMU_LSB_PER_G,
					  az / IMU_LSB_PER_G);
			if (t.jolt_g > peak_jolt) {
				peak_jolt = t.jolt_g;
			}
			if (ev != 0U) {
				print_events(ev, &t);
			}
			reads++;
		} else {
			fails++;
		}

		/* Status line, about once a second.  "reads" should stay near
		   100: it shows the loop really runs every 10 ms. */
		count++;
		if (count >= STATUS_EVERY) {
			tm_printf((UB *)"[imu] %s tilt %d.%d deg, jolt %d.%d g, reads %u, failed %u\n",
				  state_name[t.state], WHOLE(t.tilt_deg), TENTH(t.tilt_deg),
				  WHOLE(peak_jolt), TENTH(peak_jolt), (UINT)reads, (UINT)fails);
			peak_jolt = 0.0f;
			reads = 0U;
			fails = 0U;
			count = 0;
		}
	}
}

/* Kernel objects: called from subsystem4_init(). */
EXPORT ER terrain_objects_init(void)
{
	T_CCYC ccyc = {0};

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)terrain_cyc;
	ccyc.cyctim = TERRAIN_READ_MS;
	ccyc.cycphs = TERRAIN_READ_MS;
	cyc_terrain = tk_cre_cyc(&ccyc);
	return (cyc_terrain < E_OK) ? cyc_terrain : E_OK;
}

/*
 *	task_terrain.c
 *	Subsystem 4 -- Terrain Monitor task: calibration once the line sensors
 *	are done, then every 10 ms reads the GY-511, runs the hump and collision
 *	detector (terrain_logic.c) and reports what it finds:
 *	  heartbeat  HB_TERRAIN_MONITOR on flg_hb, every time it wakes
 *	  faults     FAULT_BIT_COLLISION and FAULT_BIT_IMU_FAULT on flg_faults
 *	  message    terrain_msg_t on mbf_terrain, whenever something changes
 *
 *	Why a cyclic handler: the kernel's clock ticks every 10 ms
 *	(CNF_TIMER_PERIOD), and tk_dly_tsk(10) adds one tick for safety, so a
 *	delay loop would really read every 20 ms.  The detector was tested at
 *	10 ms, so the handler wakes the task once per tick instead.
 *
 *	The task never drives the motors.  Mission Control doesn't read the
 *	faults or mbf_terrain yet, so these reports don't change how the car
 *	drives until Sean adds that.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "terrain_logic.h"
#include "../subsystem1/subsystem1.h"	/* flg_hb, HB_TERRAIN_MONITOR */
#include "../subsystem2/subsystem2.h"	/* flg_faults, terrain_pack() */
#include "../subsystem3/subsystem3.h"	/* s3_is_calibrated() */

#define READY_WAIT_MAX_S	30	/* line calibration failed: go on anyway */
#define CAL_WAIT_S		5
#define CAL_DISCARD		10	/* the first readings after start aren't good */
#define CAL_SAMPLES		20
#define CAL_GAP_MS		30
#define STATUS_EVERY		100	/* status line every 100 reads = 1 s */
#define FAIL_LIMIT		10	/* failed reads in a row = 100 ms: sensor lost */
#define TERRAIN_MSGS		4U	/* messages mbf_terrain can hold */

EXPORT ID	mbf_terrain;	/* terrain_q, read by Mission Control */

LOCAL ID	cyc_terrain;
LOCAL ID	tid_terrain;
LOCAL UW	msgs_sent = 0U;
LOCAL UW	msgs_dropped = 0U;	/* mbf_terrain full (counted, not waited for) */
LOCAL UB	last_motion = IMU_STATIONARY;	/* the latest motion event */

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

LOCAL void hb(void)
{
	(void)tk_set_flg(flg_hb, HB_TERRAIN_MONITOR);
}

/* Sleep about ms, setting the heartbeat every step.  The Safety Monitor
   counts a task as hung after 40 ms (hb_bits.h).  tk_dly_tsk(10) really
   takes 20 ms (see the top of this file), so each step counts as 20. */
LOCAL void wait_ms(UW ms)
{
	UW waited;

	for (waited = 0U; waited < ms; waited += 20U) {
		hb();
		tk_dly_tsk((RELTIM)TERRAIN_READ_MS);
	}
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
		(void)tk_set_flg(flg_faults, FAULT_BIT_IMU_FAULT);
		tm_printf((UB *)"[imu] GY-511 not found at 0x1D - check wiring\n");
		wait_ms(1000U);
	}
}

/* Wait for line calibration: the car is being moved by hand until READY. */
LOCAL void wait_for_line_calibration(void)
{
	INT s;

	for (s = 0; (s < READY_WAIT_MAX_S) && !s3_is_calibrated(); s++) {
		wait_ms(1000U);
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
	wait_ms(CAL_WAIT_S * 1000U);

	for (i = 0; i < CAL_DISCARD; i++) {
		(void)drv_imu_read(&ax, &ay, &az);
		wait_ms(CAL_GAP_MS);
	}
	for (i = 0; i < CAL_SAMPLES; i++) {
		if (drv_imu_read(&ax, &ay, &az) >= E_OK) {
			x += ax / IMU_LSB_PER_G;
			y += ay / IMU_LSB_PER_G;
			z += az / IMU_LSB_PER_G;
			good++;
		}
		wait_ms(CAL_GAP_MS);
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
	if ((ev & TERRAIN_EV_STARTED) != 0U) {
		tm_printf((UB *)"[imu] >>> MOVING\n");
	}
	if ((ev & TERRAIN_EV_STOPPED) != 0U) {
		tm_printf((UB *)"[imu] >>> STATIONARY\n");
	}
}

/* Tell Mission Control what just changed.  Zero-timeout send: the terrain
   task must never wait for the receiver.  The motion field is the LATEST
   motion event, not what the car is doing this instant.  IMU_ACCELERATING
   means "started moving from rest": with no encoders, speeding up while
   already moving can't be told from vibration.  IMU_TURNING isn't built. */
LOCAL void send_terrain(UINT ev, const terrain_t *t)
{
	BOOL const	collision = ((ev & TERRAIN_EV_COLLISION) != 0U) ? TRUE : FALSE;
	terrain_msg_t	m;

	if ((ev & TERRAIN_EV_STOPPED) != 0U) {
		last_motion = IMU_STATIONARY;
	}
	if ((ev & TERRAIN_EV_STARTED) != 0U) {
		last_motion = IMU_ACCELERATING;
	}
	if ((ev & (TERRAIN_EV_CLIMBING | TERRAIN_EV_CLIMB_AGAIN)) != 0U) {
		last_motion = IMU_CLIMBING_HUMP;
	}
	if ((ev & TERRAIN_EV_DESCENDING) != 0U) {
		last_motion = IMU_DESCENDING_HUMP;
	}
	if (collision) {
		last_motion = IMU_IMPACT;
	}
	m = terrain_pack((UB)t->state, last_motion, collision, (UB)(t->best_mm + 0.5f));

	if (tk_snd_mbf(mbf_terrain, &m, (INT)sizeof(m), TMO_POL) == E_OK) {
		msgs_sent++;
	} else {
		msgs_dropped++;
	}
	tm_printf((UB *)"[imu] terrain_q: state %u, motion %u, collision %u, highest %u mm\n",
		  (UINT)terrain_get_state(m), (UINT)terrain_get_motion(m),
		  (UINT)terrain_get_collision(m), (UINT)terrain_get_peak_mm(m));
}

/* The current bits of an event flag, for the status line. */
LOCAL UINT flag_bits(ID flg)
{
	T_RFLG rflg;

	return (tk_ref_flg(flg, &rflg) == E_OK) ? rflg.flgptn : 0U;
}

EXPORT void terrain_task(INT stacd, void *exinf)
{
	LOCAL const char * const state_name[] = { "FLAT", "CLIMBING", "DESCENDING" };
	terrain_t	t;
	float		x, y, z;
	float		peak_jolt = 0.0f;	/* biggest jolt since the last print */
	UW		reads = 0U;		/* good reads since the last print */
	UW		fails = 0U;		/* failed reads since the last print */
	UW		fails_in_row = 0U;
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
		hb();

		if (drv_imu_read(&ax, &ay, &az) >= E_OK) {
			ev = terrain_step(&t, ax / IMU_LSB_PER_G, ay / IMU_LSB_PER_G,
					  az / IMU_LSB_PER_G);
			if (t.jolt_g > peak_jolt) {
				peak_jolt = t.jolt_g;
			}
			if ((ev & TERRAIN_EV_COLLISION) != 0U) {
				(void)tk_set_flg(flg_faults, FAULT_BIT_COLLISION);
			}
			if (ev != 0U) {
				print_events(ev, &t);
				send_terrain(ev, &t);
			}
			reads++;
			fails_in_row = 0U;
		} else {
			fails++;
			fails_in_row++;
			if (fails_in_row == FAIL_LIMIT) {
				(void)tk_set_flg(flg_faults, FAULT_BIT_IMU_FAULT);
				tm_printf((UB *)"[imu] FAULT: sensor stopped answering\n");
			}
		}

		/* Status line, about once a second.  "reads" should stay near
		   100: it shows the loop really runs every 10 ms. */
		count++;
		if (count >= STATUS_EVERY) {
			tm_printf((UB *)"[imu] %s %s tilt %d.%d deg, jolt %d.%d g, shake 0.%03d g,"
				  " reads %u, failed %u, hb %x, faults %x, sent %u, dropped %u\n",
				  t.moving ? "MOVING" : "STILL", state_name[t.state],
				  WHOLE(t.tilt_deg), TENTH(t.tilt_deg),
				  WHOLE(peak_jolt), TENTH(peak_jolt),
				  (INT)((t.shake_g * 1000.0f) + 0.5f), (UINT)reads, (UINT)fails,
				  flag_bits(flg_hb) & HB_TERRAIN_MONITOR, flag_bits(flg_faults),
				  (UINT)msgs_sent, (UINT)msgs_dropped);
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
	T_CCYC	ccyc = {0};
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)((sizeof(terrain_msg_t) + 8U) * TERRAIN_MSGS);
	cmbf.maxmsz = (INT)sizeof(terrain_msg_t);
	mbf_terrain = tk_cre_mbf(&cmbf);
	if (mbf_terrain < E_OK) {
		return mbf_terrain;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)terrain_cyc;
	ccyc.cyctim = TERRAIN_READ_MS;
	ccyc.cycphs = TERRAIN_READ_MS;
	cyc_terrain = tk_cre_cyc(&ccyc);
	return (cyc_terrain < E_OK) ? cyc_terrain : E_OK;
}

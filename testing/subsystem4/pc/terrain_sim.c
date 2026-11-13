/*
 *	terrain_sim.c
 *	Laptop test for subsystem 4's detector (terrain_logic.c): no car needed.
 *	Feeds made-up sensor readings, one every 10 ms, and checks the events.
 *
 *	Build and run:  sh testing/subsystem4/pc/run.sh
 *
 *	What it can show: the logic does what it is meant to with readings
 *	shaped like the ones measured on the car (flat reading, shake size and
 *	hump tilt are from testing/subsystem4 logs of 1-2 Nov).
 *	What it can NOT show: that the limits suit the real car.  The lift and
 *	overturn readings here are made up from physics, not recorded.
 *
 *	Test IDs: S4-P01 ... (P = PC), listed in testing/subsystem4/TEST_LOG.md.
 */

#include <stdio.h>
#include <math.h>
#include "../../../app_program/subsystem4/terrain_logic.h"

/* The car's reading when flat and still (chip face down), in g */
#define FLAT_X		0.02f
#define FLAT_Y		(-0.02f)
#define FLAT_Z		(-0.91f)
#define G_MS2		9.81f
#define DT		0.01f
#define PI		3.14159265f

static terrain_t	t;
static unsigned int	seen;		/* every event since the last clear */
static int		first_lift_ms;	/* time of the first LIFTED / OVERTURNED */
static int		now_ms;
static unsigned int	rng = 12345U;
static int		passed, failed;
static int		w_ok;		/* wheel data supplied? */
static float		w_l, w_r;	/* wheel speeds, mm/s */

/* Repeatable noise, -1 to +1 */
static float noise(void)
{
	rng = rng * 1664525U + 1013904223U;
	return ((float)((rng >> 8) & 0xFFFFU) / 32767.5f) - 1.0f;
}

/* One reading: the car rolled by roll_deg (right side up) and pitched by
   pitch_deg, pushed upward at up_ms2, shaking by shake_g on every axis. */
static void feed(float roll_deg, float pitch_deg, float up_ms2, float shake_g)
{
	float const r = roll_deg * PI / 180.0f;
	float const p = pitch_deg * PI / 180.0f;
	float const k = 1.0f + (up_ms2 / G_MS2);	/* pushed up = heavier */
	float x = FLAT_X, y = FLAT_Y, z = FLAT_Z;
	float y2, z2, x2;
	unsigned int ev;

	y2 = y * cosf(r) - z * sinf(r);		/* roll: about the X axis */
	z2 = y * sinf(r) + z * cosf(r);
	x2 = x * cosf(p) - z2 * sinf(p);	/* pitch: about the Y axis */
	z2 = x * sinf(p) + z2 * cosf(p);

	terrain_wheels(&t, w_ok, w_l, w_r);
	ev = terrain_step(&t, x2 * k + shake_g * noise(), y2 * k + shake_g * noise(),
			  z2 * k + shake_g * noise());
	seen |= ev;
	now_ms += 10;
	if ((first_lift_ms < 0) && ((ev & (TERRAIN_EV_LIFTED | TERRAIN_EV_OVERTURNED)) != 0U)) {
		first_lift_ms = now_ms;
	}
}

static void run(int ms, float roll, float pitch, float up, float shake)
{
	int i;

	for (i = 0; i < ms; i += 10) {
		feed(roll, pitch, up, shake);
	}
}

/* Roll smoothly from one angle to another */
static void roll_to(int ms, float from, float to, float shake)
{
	int i;

	for (i = 0; i < ms; i += 10) {
		feed(from + (to - from) * (float)i / (float)ms, 0.0f, 0.0f, shake);
	}
}

/* Lift straight up by cm in ms: speed up for the first half, slow down for
   the second.  Returns nothing; the events are in "seen". */
static void lift(int ms, float cm, float shake)
{
	float const T = (float)ms / 1000.0f;
	float const a = 4.0f * (cm / 100.0f) / (T * T);
	int i;

	for (i = 0; i < ms; i += 10) {
		feed(0.0f, 0.0f, (i < ms / 2) ? a : -a, shake);
	}
}

/* Drive over the black ramp: right side tips up to peak_deg and back, and
   the sensor rises rise_mm on the way. */
static void hump(int ms, float peak_deg, float rise_mm, float shake)
{
	float const T = (float)ms / 2000.0f;		/* time to the top */
	float const a = 4.0f * (rise_mm / 1000.0f) / (T * T);
	int i;

	for (i = 0; i < ms; i += 10) {
		float const f  = (float)i / (float)ms;
		float const up = (f < 0.25f) ? a : (f < 0.75f) ? -a : a;

		feed(peak_deg * sinf(PI * f), 0.0f, up, shake);
	}
}

static void start(void)
{
	terrain_init(&t, FLAT_X, FLAT_Y, FLAT_Z);
	w_ok = 0;
	w_l = 0.0f;
	w_r = 0.0f;
	seen = 0U;
	first_lift_ms = -1;
	now_ms = 0;
	rng = 12345U;
}

static void mark(void)
{
	seen = 0U;
	first_lift_ms = -1;
	now_ms = 0;
	t.up_speed_max = 0.0f;
}

static void check(const char *id, int ok, const char *what)
{
	printf("[TEST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (ok) {
		passed++;
	} else {
		failed++;
	}
}

#define OFF_FLOOR	(TERRAIN_EV_LIFTED | TERRAIN_EV_OVERTURNED)
/* Random shake per axis.  This model OVER-states how much shaking adds to
   the up speed: on the car (13 Nov) driving shook about 0.05 g and the up
   speed stayed at or under 0.02 m/s, but random noise of 0.05 g gives
   about 0.05 m/s here (real vibration cancels out better).  So the noise
   is sized by what matters for the lift check, the up speed:
     DRIVE_SHAKE gives about 0.02 m/s, as measured while driving;
     FAST_SHAKE  gives about 0.04 m/s, twice that, as a margin, because
     FAST speed has not been measured with this code. */
#define DRIVE_SHAKE	0.02f
#define FAST_SHAKE	0.04f

int main(void)
{
	float best;

	/* ---- must NOT say lifted ---- */
	start();
	run(5000, 0, 0, 0, 0.005f);
	check("S4-P01", (seen & OFF_FLOOR) == 0U, "still for 5 s: not lifted");

	start();
	run(30000, 0, 0, 0, DRIVE_SHAKE);
	printf("        highest up-speed %.3f m/s (limit 0.06)\n", t.up_speed_max);
	check("S4-P02", (seen & OFF_FLOOR) == 0U, "driving 30 s, shake as measured: not lifted");

	start();
	run(30000, 0, 0, 0, FAST_SHAKE);
	printf("        highest up-speed %.3f m/s (limit 0.06)\n", t.up_speed_max);
	check("S4-P03", (seen & OFF_FLOOR) == 0U, "driving 30 s, twice the measured up-speed noise: not lifted");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	hump(1000, 7.0f, 5.0f, DRIVE_SHAKE);
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	printf("        hump height %.1f mm, highest up-speed %.3f m/s\n",
	       t.height_mm, t.up_speed_max);
	check("S4-P04", ((seen & OFF_FLOOR) == 0U) && ((seen & TERRAIN_EV_HUMP_DONE) != 0U) &&
	      (t.height_mm > 9.0f) && (t.height_mm < 12.0f),
	      "ramp at 7 deg in 1 s: hump counted (9-12 mm), not lifted");

	start();
	run(2000, 0, 0, 0, FAST_SHAKE);
	hump(400, 7.0f, 5.0f, FAST_SHAKE);
	run(1000, 0, 0, 0, FAST_SHAKE);
	printf("        highest up-speed %.3f m/s\n", t.up_speed_max);
	check("S4-P05", (seen & OFF_FLOOR) == 0U, "ramp at 7 deg in 0.4 s, twice the measured noise: not lifted");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	feed(0, 0, 0.9f * G_MS2, DRIVE_SHAKE);		/* one reading, 0.9 g upward */
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	feed(20.0f, 0, 0, 2.0f);			/* one wild reading */
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P06", (seen & OFF_FLOOR) == 0U, "one hard bump from below, one wild reading: not lifted");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	run(300, 0, 12.0f, 0, DRIVE_SHAKE);		/* nose dips or lifts 12 deg */
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P07", (seen & OFF_FLOOR) == 0U, "12 deg pitch for 0.3 s (hard braking): not lifted");

	/* ---- MUST say lifted or overturned ---- */
	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	mark();
	lift(600, 20.0f, DRIVE_SHAKE);
	run(500, 0, 0, 0, 0.05f);
	printf("        detected after %d ms\n", first_lift_ms);
	check("S4-P08", ((seen & TERRAIN_EV_LIFTED) != 0U) && (first_lift_ms <= 300),
	      "lifted level 20 cm in 0.6 s while driving: LIFTED within 300 ms");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	mark();
	lift(1000, 20.0f, DRIVE_SHAKE);		/* slow and level: a known limit */
	printf("        slow level lift alone: %s (highest up-speed %.3f m/s)\n",
	       ((seen & OFF_FLOOR) != 0U) ? "detected" : "NOT detected", t.up_speed_max);
	roll_to(300, 0.0f, 35.0f, 0.05f);		/* ... then tipped in the hand */
	run(500, 35.0f, 0, 0, 0.05f);
	printf("        detected after %d ms\n", first_lift_ms);
	check("S4-P09", (seen & TERRAIN_EV_LIFTED) != 0U,
	      "lifted slowly (20 cm in 1 s), then tipped 35 deg: LIFTED once tipped");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	mark();
	roll_to(300, 0.0f, 35.0f, DRIVE_SHAKE);
	run(700, 35.0f, 0, 0, 0.05f);
	printf("        detected after %d ms\n", first_lift_ms);
	check("S4-P10", ((seen & TERRAIN_EV_LIFTED) != 0U) && (first_lift_ms <= 600),
	      "tipped to 35 deg and held: LIFTED within 600 ms");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	mark();
	roll_to(300, 0.0f, 90.0f, DRIVE_SHAKE);
	run(700, 90.0f, 0, 0, 0.02f);
	printf("        detected after %d ms\n", first_lift_ms);
	check("S4-P11", ((seen & TERRAIN_EV_OVERTURNED) != 0U) && (first_lift_ms <= 500),
	      "rolled onto its side: off the floor within 500 ms, then OVERTURNED");

	start();
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	mark();
	roll_to(400, 0.0f, 180.0f, DRIVE_SHAKE);
	run(700, 180.0f, 0, 0, 0.02f);
	check("S4-P12", (seen & TERRAIN_EV_OVERTURNED) != 0U, "flipped upside down: OVERTURNED");

	/* ---- after a lift ---- */
	start();
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	hump(1000, 7.0f, 5.0f, DRIVE_SHAKE);
	run(500, 0, 0, 0, DRIVE_SHAKE);
	best = t.best_mm;
	lift(600, 20.0f, DRIVE_SHAKE);
	mark();
	hump(1000, 15.0f, 0.0f, 0.06f);			/* waved about in the hand */
	hump(1000, 12.0f, 0.0f, 0.06f);
	check("S4-P13", ((seen & (TERRAIN_EV_HUMP_DONE | TERRAIN_EV_CLIMBING)) == 0U) &&
	      (t.best_mm == best) && (best > 9.0f),
	      "tilted about while carried: no hump counted, highest height unchanged");

	mark();
	run(2500, 0, 0, 0, 0.06f);			/* held level but not still */
	check("S4-P14", (seen & TERRAIN_EV_PUT_DOWN) == 0U, "held level in a shaky hand 2.5 s: still off the floor");

	mark();
	run(2500, 2.0f, 0, 0, 0.005f);			/* down on a floor 2 deg off */
	check("S4-P15", ((seen & TERRAIN_EV_PUT_DOWN) != 0U) && (t.lift == TERRAIN_ON_FLOOR),
	      "put down level and still: PUT_DOWN within 2.5 s");

	mark();
	run(1000, 2.0f, 0, 0, DRIVE_SHAKE);
	hump(1000, 9.0f, 5.0f, DRIVE_SHAKE);		/* 2 deg floor + 7 deg hump */
	run(1000, 2.0f, 0, 0, DRIVE_SHAKE);
	printf("        hump height %.1f mm\n", t.height_mm);
	check("S4-P16", ((seen & TERRAIN_EV_HUMP_DONE) != 0U) && ((seen & OFF_FLOOR) == 0U),
	      "driven over the ramp after being put down: hump counted again");

	/* ---- wheel encoders (speeds as Sean's odometry gives them) ---- */
	start();
	w_ok = 1;
	run(1000, 0, 0, 0, 0.005f);
	check("S4-P17", ((seen & TERRAIN_EV_STARTED) == 0U) && (t.moving == 0),
	      "wheels stopped, no shake: stationary");
	w_l = 150.0f; w_r = 150.0f;
	mark();
	run(100, 0, 0, 0, 0.005f);
	check("S4-P18", ((seen & TERRAIN_EV_STARTED) != 0U) && (t.moving == 1),
	      "wheels turn on a smooth floor (no shake): MOVING within 100 ms");
	w_l = 0.0f; w_r = 0.0f;
	mark();
	run(1500, 0, 0, 0, 0.005f);
	check("S4-P19", ((seen & TERRAIN_EV_STOPPED) != 0U) && (t.moving == 0),
	      "wheels stop and the shake dies down: STATIONARY");

	start();
	w_ok = 1;				/* encoders unplugged: they read 0 */
	run(2000, 0, 0, 0, 0.05f);		/* shake as measured on the car */
	check("S4-P20", ((seen & TERRAIN_EV_STARTED) != 0U) && ((seen & TERRAIN_EV_TURN_START) == 0U),
	      "encoders read 0 but the car shakes: still MOVING, never TURNING");

	start();
	w_ok = 1; w_l = 150.0f; w_r = 150.0f;
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P21", (seen & TERRAIN_EV_TURN_START) == 0U, "both wheels 150 mm/s: not turning");
	w_l = 60.0f; w_r = 240.0f;		/* (240-60)/110 rad/s = 94 deg/s to the left */
	mark();
	run(600, 0, 0, 0, DRIVE_SHAKE);
	printf("        turn rate %.0f deg/s\n", t.turn_dps);
	check("S4-P22", ((seen & TERRAIN_EV_TURN_START) != 0U) && (t.turn_dps > 85.0f) &&
	      (t.turn_dps < 100.0f), "left 60, right 240 mm/s: TURNING left at about 94 deg/s");
	w_l = 150.0f; w_r = 150.0f;
	mark();
	run(600, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P23", (seen & TERRAIN_EV_TURN_END) != 0U, "wheels equal again: TURN ENDED");
	w_l = 240.0f; w_r = 60.0f;
	mark();
	run(100, 0, 0, 0, DRIVE_SHAKE);		/* a 100 ms steering correction */
	w_l = 150.0f; w_r = 150.0f;
	run(500, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P24", (seen & TERRAIN_EV_TURN_START) == 0U,
	      "a 100 ms steering correction: not counted as a turn");

	start();
	w_ok = 1; w_l = 150.0f; w_r = 150.0f;
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	lift(600, 20.0f, DRIVE_SHAKE);
	w_l = 0.0f; w_r = 400.0f;		/* one wheel spinning free in the air */
	mark();
	run(1000, 0, 0, 0, 0.05f);
	check("S4-P25", ((seen & TERRAIN_EV_TURN_START) == 0U) && (t.lift != TERRAIN_ON_FLOOR),
	      "wheels spinning unevenly while lifted: not counted as a turn");

	start();
	w_ok = 1; w_l = 150.0f; w_r = 0.0f;	/* right encoder dead from the start */
	run(2000, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P26", (seen & TERRAIN_EV_TURN_START) == 0U,
	      "one encoder never moves: not counted as a turn");

	start();
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	lift(600, 20.0f, DRIVE_SHAKE);
	mark();
	run(4000, 6.0f, 0, 0, 0.06f);		/* held in a hand 6 deg off level, 4 s */
	check("S4-P28", (seen & TERRAIN_EV_PUT_DOWN) == 0U,
	      "held in a hand about 6 deg off level for 4 s: still off the floor");
	mark();
	run(3500, 0, 0, 0, DRIVE_SHAKE);		/* put down and started at once */
	hump(1000, 7.0f, 5.0f, DRIVE_SHAKE);
	run(1000, 0, 0, 0, DRIVE_SHAKE);
	check("S4-P27", ((seen & TERRAIN_EV_PUT_DOWN) != 0U) && ((seen & TERRAIN_EV_HUMP_DONE) != 0U),
	      "put down and driven off at once: back on the floor within 3.5 s, next hump counted");

	printf("\n%d passed, %d failed\n", passed, failed);
	return (failed == 0) ? 0 : 1;
}

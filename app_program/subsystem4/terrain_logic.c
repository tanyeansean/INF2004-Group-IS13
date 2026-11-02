/*
 *	terrain_logic.c
 *	Subsystem 4 -- hump and collision detector (Buddy 4, Jin Hin).
 *
 *	Same logic and settings as hump detector v5 (imu_test_v5.c), which
 *	passed on the black ramp (19.8 cm long, about 1 cm high) on 1 Nov.
 *	States: FLAT -> CLIMBING -> DESCENDING -> FLAT.
 *
 *	Mounting: chip FACE DOWN, X arrow pointing to the car's front.
 *	  X = forward, Y = right, Z = down (so Z reads about -0.9 when flat).
 *
 *	Hump height: the ramp lifts the right wheel and the car tips over like
 *	a hinge, so height = D_MM x sin(highest tilt).
 *
 *	Tilt is measured against a "flat" direction that slowly follows the
 *	floor, so a sloped or uneven floor doesn't add to the hump height.
 *
 *	The team's build doesn't link the maths library, so the three maths
 *	functions it would give are at the top of this file.
 */

#include "terrain_logic.h"

/* ---- Collision settings (tune these) ---- */
#define COLLISION_JOLT	1.0f	/* jolt above this = collision (g) */
#define BLANK_MS	500	/* ignore tilt this long after a collision */
#define GRAVITY_SMOOTH	0.05f	/* averaging: new reading counts 5% (about 0.2 s) */

/* ---- "Flat" reference: follows the floor slowly, only while near flat ---- */
#define FLAT_SMOOTH	0.005f	/* new reading counts 0.5% (about 2 s) */
#define FLAT_TRACK_DEG	1.5f	/* only follow the floor when tilt is below this */

/* ---- Hump detector settings (from the black ramp runs, 1 Nov) ---- */
#define CLIMB_DEG	2.5f	/* tilt above this ...                    */
#define CLIMB_MS	100	/* ... for this long = start of a hump    */
#define DROP_DEG	1.0f	/* tilt this far below its highest = going down */
#define FLAT_DEG	1.5f	/* tilt below this ...                    */
#define FLAT_MS		100	/* ... for this long = hump finished      */
#define MIN_PEAK_DEG	3.5f	/* a smaller peak isn't counted as a hump */
#define MAX_PEAK_DEG	20.0f	/* a bigger peak is the car being lifted  */
#define HUMP_TIMEOUT_MS	5000	/* stuck this long = not a hump, start again */
#define D_MM		87.0f	/* from still tests at 10, 20, 30 mm, 1 Nov (tape, +/-1 mm) */

/* ---- Moving or stationary: a driving car shakes, a stopped one doesn't.
   Provisional numbers until they are measured on the car. ---- */
#define SHAKE_SMOOTH	0.02f	/* averaging: new jolt counts 2% (about 0.5 s) */
#define MOVING_SHAKE	0.03f	/* shake above this ...                   */
#define MOVING_MS	200	/* ... for this long = the car started    */
#define STILL_SHAKE	0.02f	/* shake below this ...                   */
#define STILL_MS	500	/* ... for this long = the car stopped    */

#define RAD_TO_DEG	57.2958f

/* Square root: keep averaging the guess with x / guess (Newton's method). */
static float root(float x)
{
	float	guess = (x > 1.0f) ? x : 1.0f;
	int	i;

	if (x <= 0.0f) {
		return 0.0f;
	}
	for (i = 0; i < 20; i++) {
		guess = 0.5f * (guess + (x / guess));
	}
	return guess;
}

/* Arctangent in degrees for r from 0 to 1 (a standard polynomial fit). */
static float atan_deg(float r)
{
	float const r2 = r * r;

	return RAD_TO_DEG * r * (0.9998660f + r2 * (-0.3302995f + r2 * (0.1801410f +
	       r2 * (-0.0851330f + r2 * 0.0208351f))));
}

/* The angle whose sideways part is "side" (never negative) and whose along
   part is "along": 0 to 180 degrees. */
static float angle_deg(float side, float along)
{
	float const	along_abs = (along < 0.0f) ? -along : along;
	float		a;

	if ((side == 0.0f) && (along_abs == 0.0f)) {
		return 0.0f;
	}
	if (side <= along_abs) {
		a = atan_deg(side / along_abs);
	} else {
		a = 90.0f - atan_deg(along_abs / side);
	}
	return (along < 0.0f) ? (180.0f - a) : a;
}

/* Sine of an angle in degrees, for the hump range (0 to 20 degrees). */
static float sin_deg(float deg)
{
	float const x  = deg / RAD_TO_DEG;
	float const x2 = x * x;

	return x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f)));
}

/* Angle in degrees between two directions (a and b).  The cross product
   gives the sideways part and the dot product the along part. */
static float angle_between(float ax, float ay, float az,
			   float bx, float by, float bz)
{
	float const cx = ay*bz - az*by;
	float const cy = az*bx - ax*bz;
	float const cz = ax*by - ay*bx;

	return angle_deg(root(cx*cx + cy*cy + cz*cz), ax*bx + ay*by + az*bz);
}

void terrain_init(terrain_t *t, float x_g, float y_g, float z_g)
{
	t->state = TERRAIN_FLAT;
	t->grav_x = x_g;  t->grav_y = y_g;  t->grav_z = z_g;
	t->flat_x = x_g;  t->flat_y = y_g;  t->flat_z = z_g;
	t->tilt_deg = 0.0f;
	t->jolt_g = 0.0f;
	t->shake_g = 0.0f;
	t->moving = 0;
	t->shake_ms = 0;
	t->hump_peak_deg = 0.0f;
	t->height_mm = 0.0f;
	t->best_mm = 0.0f;
	t->timer_ms = 0;
	t->hump_ms = 0;
	t->blank_ms = 0;
}

void terrain_new_run(terrain_t *t)
{
	t->best_mm = 0.0f;
}

/* The hump state machine: one step per reading. */
static unsigned int hump_step(terrain_t *t)
{
	float const	tilt = t->tilt_deg;
	unsigned int	ev = 0U;

	switch (t->state) {
	case TERRAIN_FLAT:
		/* Near flat: slowly follow the floor's slope */
		if (tilt < FLAT_TRACK_DEG) {
			t->flat_x += FLAT_SMOOTH * (t->grav_x - t->flat_x);
			t->flat_y += FLAT_SMOOTH * (t->grav_y - t->flat_y);
			t->flat_z += FLAT_SMOOTH * (t->grav_z - t->flat_z);
		}
		/* Start of a hump only if the tilt STAYS above CLIMB_DEG */
		if (tilt > CLIMB_DEG) {
			t->timer_ms += TERRAIN_READ_MS;
			if (t->timer_ms >= CLIMB_MS) {
				t->state = TERRAIN_CLIMBING;
				t->hump_peak_deg = tilt;
				t->hump_ms = 0;
				t->timer_ms = 0;
				ev |= TERRAIN_EV_CLIMBING;
			}
		} else {
			t->timer_ms = 0;
		}
		break;

	case TERRAIN_CLIMBING:
		/* Remember the highest tilt; once it drops a bit, we're past the top */
		if (tilt > t->hump_peak_deg) {
			t->hump_peak_deg = tilt;
		}
		if (tilt < (t->hump_peak_deg - DROP_DEG)) {
			t->state = TERRAIN_DESCENDING;
			t->timer_ms = 0;
			ev |= TERRAIN_EV_DESCENDING;
		}
		break;

	case TERRAIN_DESCENDING:
		if (tilt > t->hump_peak_deg) {
			/* Went higher after all: we weren't past the top yet */
			t->hump_peak_deg = tilt;
			t->state = TERRAIN_CLIMBING;
			t->timer_ms = 0;
			ev |= TERRAIN_EV_CLIMB_AGAIN;
		} else if (tilt < FLAT_DEG) {
			/* Hump is over once the tilt STAYS low */
			t->timer_ms += TERRAIN_READ_MS;
			if (t->timer_ms >= FLAT_MS) {
				if ((t->hump_peak_deg >= MIN_PEAK_DEG) &&
				    (t->hump_peak_deg <= MAX_PEAK_DEG)) {
					t->height_mm = D_MM * sin_deg(t->hump_peak_deg);
					if (t->height_mm > t->best_mm) {
						t->best_mm = t->height_mm;
					}
					ev |= TERRAIN_EV_HUMP_DONE;
				} else {
					ev |= TERRAIN_EV_NOT_A_HUMP;
				}
				t->state = TERRAIN_FLAT;
				t->timer_ms = 0;
			}
		} else {
			t->timer_ms = 0;
		}
		break;
	}

	/* Stuck in a hump for too long?  It wasn't a hump (e.g. the car was
	   put down on a slope), so treat this position as the new flat. */
	if (t->state != TERRAIN_FLAT) {
		t->hump_ms += TERRAIN_READ_MS;
		if (t->hump_ms >= HUMP_TIMEOUT_MS) {
			t->flat_x = t->grav_x;
			t->flat_y = t->grav_y;
			t->flat_z = t->grav_z;
			t->state = TERRAIN_FLAT;
			t->timer_ms = 0;
			ev |= TERRAIN_EV_TIMED_OUT;
		}
	}
	return ev;
}

/* Has the car started or stopped?  Two different limits, so a shake close to
   one limit can't flip the answer back and forth. */
static unsigned int motion_step(terrain_t *t)
{
	unsigned int ev = 0U;

	t->shake_g += SHAKE_SMOOTH * (t->jolt_g - t->shake_g);

	if ((t->moving == 0) && (t->shake_g > MOVING_SHAKE)) {
		t->shake_ms += TERRAIN_READ_MS;
		if (t->shake_ms >= MOVING_MS) {
			t->moving = 1;
			t->shake_ms = 0;
			ev |= TERRAIN_EV_STARTED;
		}
	} else if ((t->moving != 0) && (t->shake_g < STILL_SHAKE)) {
		t->shake_ms += TERRAIN_READ_MS;
		if (t->shake_ms >= STILL_MS) {
			t->moving = 0;
			t->shake_ms = 0;
			ev |= TERRAIN_EV_STOPPED;
		}
	} else {
		t->shake_ms = 0;
	}
	return ev;
}

unsigned int terrain_step(terrain_t *t, float x_g, float y_g, float z_g)
{
	/* Jolt = how far this reading is from the averaged gravity */
	float const	dx = x_g - t->grav_x;
	float const	dy = y_g - t->grav_y;
	float const	dz = z_g - t->grav_z;
	unsigned int	ev = 0U;

	t->jolt_g = root(dx*dx + dy*dy + dz*dz);
	ev |= motion_step(t);
	if (t->jolt_g > COLLISION_JOLT) {
		if (t->blank_ms <= 0) {
			ev |= TERRAIN_EV_COLLISION;
		}
		t->blank_ms = BLANK_MS;
	}
	if (t->blank_ms > 0) {
		t->blank_ms -= TERRAIN_READ_MS;
	}

	/* After a big hit, skip this reading so the hit doesn't bend the average */
	if (t->blank_ms <= 0) {
		t->grav_x += GRAVITY_SMOOTH * dx;
		t->grav_y += GRAVITY_SMOOTH * dy;
		t->grav_z += GRAVITY_SMOOTH * dz;

		/* Tilt is measured against the floor the car is on right now */
		t->tilt_deg = angle_between(t->grav_x, t->grav_y, t->grav_z,
					    t->flat_x, t->flat_y, t->flat_z);
		ev |= hump_step(t);
	}
	return ev;
}

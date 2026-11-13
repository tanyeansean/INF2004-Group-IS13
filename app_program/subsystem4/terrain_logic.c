/*
 *	terrain_logic.c
 *	Subsystem 4 -- hump detector and lifted / overturned detector
 *	(Buddy 4, Jin Hin).
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
 *	Lifted or overturned (replaces collision detection, requirement change
 *	of 13 Nov): the car counts as off the floor when ANY of these holds:
 *	  1. it moves straight up faster than a hump could push it
 *	     (speed = the extra force along gravity, added up over time).
 *	     On the car (13 Nov) driving and the ramp never went above
 *	     0.02 m/s, and hand pick-ups reached 0.08-0.24 m/s;
 *	  2. it leans further than any hump could tip it, and stays there;
 *	  3. it leans so far that it is on its side or upside down.
 *	It stays "off the floor" until it has been level and still for a
 *	second.  While it is off the floor no humps are counted, so carrying
 *	the car can't add a hump or raise the highest height.
 *
 *	The team's build doesn't link the maths library, so the three maths
 *	functions it would give are at the top of this file.
 */

#include "terrain_logic.h"

/* ---- Hard knocks: kept out of the gravity average ---- */
#define KNOCK_JOLT	1.0f	/* jolt above this = a hard knock (g) */
#define BLANK_MS	500	/* ignore hump tilt this long after a knock */
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

/* ---- Moving or stationary.  Two signs, either one is enough for
   "moving": the car shakes (IMU), or a wheel turns (encoders).  It is
   "stationary" only when both are quiet.  So the answer is still right
   if the encoders are unplugged (they read 0) or the floor is smooth.
   Provisional numbers until they are measured on the car. ---- */
#define SHAKE_SMOOTH	0.02f	/* averaging: new jolt counts 2% (about 0.5 s) */
#define MOVING_SHAKE	0.03f	/* shake above this ...                   */
#define MOVING_MS	200	/* ... for this long = the car started    */
#define STILL_SHAKE	0.02f	/* shake below this ...                   */
#define STILL_MS	500	/* ... for this long = the car stopped    */
#define WHEEL_MOVING_MMS	20.0f	/* a wheel faster than this = rolling     */
#define WHEEL_STOP_MS	200	/* both wheels slower for this long = not rolling */

/* ---- Turning, from the wheels: the outside wheel goes faster.
   turn rate = (right speed - left speed) / distance between the wheels.
   TRACK_MM: middle of the left tyre to middle of the right tyre.
   Measured by Jin on 13 Nov with a tape: 125 mm outside edge to outside
   edge, minus one tyre width (15 mm) = 110 mm, about +/-2 mm.  The other
   numbers are provisional. ---- */
#define TRACK_MM	110.0f	/* measured 13 Nov */
#define TURN_SMOOTH	0.2f	/* averaging: new reading counts 20% (about 50 ms) */
#define TURN_DPS	45.0f	/* turning faster than this (deg/s) ...   */
#define TURN_MS		150	/* ... for this long = turning            */
#define TURN_END_DPS	25.0f	/* turning slower than this ...           */
#define TURN_END_MS	200	/* ... for this long = no longer turning  */

/* ---- Lifted / overturned.  First tuned on the car on 13 Nov (straight
   track with the black ramp, power 36 and 45, logs in Jin's vault):
     driving and ramp, 95 s:  up speed at most 0.02 m/s, lean at most 7.3 deg
     6 hand pick-ups:        up speed 0.08-0.24 m/s, lean 10-30 deg
   The old up-speed limit (0.35 m/s, from the laptop model) missed 3 of
   those 6 pick-ups, so it is now 0.06 m/s: 3x the most seen while
   driving, and under the gentlest pick-up.  Curves and FAST speed are
   NOT measured yet (TEST_LOG S4-L08, S4-L11). ---- */
#define QUICK_SMOOTH	0.10f	/* averaging: new reading counts 10% (about 0.1 s) */
#define LIFT_UP_SPEED	0.06f	/* moving up faster than this (m/s) ...   */
#define LIFT_UP_MS	50	/* ... for this long = lifted             */
#define LIFT_LEAN_DEG	25.0f	/* leaning more than this ...             */
#define LIFT_LEAN_MS	150	/* ... for this long = lifted             */
#define OVERTURN_DEG	60.0f	/* leaning more than this ...             */
#define OVERTURN_MS	100	/* ... for this long = overturned         */
#define PUT_DOWN_DEG	10.0f	/* leaning less than this, and ...        */
#define PUT_DOWN_SHAKE	0.02f	/* ... shaking less than this (g) ...     */
#define PUT_DOWN_MS	1000	/* ... for this long = back on the floor  */
#define PUT_DOWN_LEVEL_MS 3000	/* or very level for this long, shaking or not */
#define PUT_DOWN_LEVEL_DEG 4.0f	/* "very level": a hand holds the car 5-16 deg off
				   (car test 13 Nov), driving reads under 2 deg */
#define UP_REF_SMOOTH	0.005f	/* the usual force along gravity follows slowly (2 s) */
#define UP_FADE		0.01f	/* up speed fades 1% per reading (about 1 s) */
#define UP_STEP_MAX_G	0.3f	/* one reading adds at most this much push (g) */
#define G_MS2		9.81f	/* 1 g in m/s2 */
#define READ_S		(TERRAIN_READ_MS / 1000.0f)

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
	t->shaking = 0;
	t->shake_ms = 0;
	t->wheels_ok = 0;
	t->wheels_seen = 0;
	t->wheel_l = 0.0f;
	t->wheel_r = 0.0f;
	t->rolling = 0;
	t->roll_ms = 0;
	t->turn_dps = 0.0f;
	t->turning = 0;
	t->turn_ms = 0;
	t->humps = 0;
	t->hump_peak_deg = 0.0f;
	t->height_mm = 0.0f;
	t->best_mm = 0.0f;
	t->timer_ms = 0;
	t->hump_ms = 0;
	t->blank_ms = 0;
	t->lift = TERRAIN_ON_FLOOR;
	t->quick_x = x_g;  t->quick_y = y_g;  t->quick_z = z_g;
	t->lean_deg = 0.0f;
	t->up_ref_g = root(x_g*x_g + y_g*y_g + z_g*z_g);
	if (t->up_ref_g < 0.5f) {
		t->up_ref_g = 1.0f;	/* calibration failed: assume 1 g */
	}
	t->up_speed = 0.0f;
	t->up_speed_max = 0.0f;
	t->up_ms = 0;
	t->lean_ms = 0;
	t->over_ms = 0;
	t->down_ms = 0;
	t->level_ms = 0;
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
				t->humps++;
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

static float absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

void terrain_wheels(terrain_t *t, int ok, float left_mms, float right_mms)
{
	t->wheels_ok = ok;
	t->wheel_l = ok ? left_mms : 0.0f;
	t->wheel_r = ok ? right_mms : 0.0f;
	if (absf(t->wheel_l) > WHEEL_MOVING_MMS) {
		t->wheels_seen |= 1;
	}
	if (absf(t->wheel_r) > WHEEL_MOVING_MMS) {
		t->wheels_seen |= 2;
	}
}

/* Has the car started or stopped?  The shake has two different limits, so a
   shake close to one limit can't flip the answer back and forth. */
static unsigned int motion_step(terrain_t *t)
{
	unsigned int	ev = 0U;
	int		now_moving;

	/* Sign 1: the car is shaking (IMU) */
	t->shake_g += SHAKE_SMOOTH * (t->jolt_g - t->shake_g);

	if ((t->shaking == 0) && (t->shake_g > MOVING_SHAKE)) {
		t->shake_ms += TERRAIN_READ_MS;
		if (t->shake_ms >= MOVING_MS) {
			t->shaking = 1;
			t->shake_ms = 0;
		}
	} else if ((t->shaking != 0) && (t->shake_g < STILL_SHAKE)) {
		t->shake_ms += TERRAIN_READ_MS;
		if (t->shake_ms >= STILL_MS) {
			t->shaking = 0;
			t->shake_ms = 0;
		}
	} else {
		t->shake_ms = 0;
	}

	/* Sign 2: a wheel is turning (encoders) */
	if ((absf(t->wheel_l) > WHEEL_MOVING_MMS) || (absf(t->wheel_r) > WHEEL_MOVING_MMS)) {
		t->rolling = 1;
		t->roll_ms = 0;
	} else if (t->rolling != 0) {
		t->roll_ms += TERRAIN_READ_MS;
		if (t->roll_ms >= WHEEL_STOP_MS) {
			t->rolling = 0;
		}
	} else {
		/* not rolling */
	}

	now_moving = ((t->shaking != 0) || (t->rolling != 0)) ? 1 : 0;
	if (now_moving != t->moving) {
		t->moving = now_moving;
		ev |= now_moving ? TERRAIN_EV_STARTED : TERRAIN_EV_STOPPED;
	}
	return ev;
}

/* Is the car turning?  Only from wheels that have both been seen to turn
   (a dead encoder would look like a hard turn), and only on the floor
   (wheels in the air say nothing about the car). */
static unsigned int turn_step(terrain_t *t)
{
	unsigned int	ev = 0U;
	float		rate = 0.0f;
	float		size;

	if ((t->wheels_ok != 0) && (t->wheels_seen == 3) && (t->lift == TERRAIN_ON_FLOOR)) {
		rate = ((t->wheel_r - t->wheel_l) / TRACK_MM) * RAD_TO_DEG;
	}
	t->turn_dps += TURN_SMOOTH * (rate - t->turn_dps);
	size = absf(t->turn_dps);

	if ((t->turning == 0) && (size > TURN_DPS)) {
		t->turn_ms += TERRAIN_READ_MS;
		if (t->turn_ms >= TURN_MS) {
			t->turning = 1;
			t->turn_ms = 0;
			ev |= TERRAIN_EV_TURN_START;
		}
	} else if ((t->turning != 0) && (size < TURN_END_DPS)) {
		t->turn_ms += TERRAIN_READ_MS;
		if (t->turn_ms >= TURN_END_MS) {
			t->turning = 0;
			t->turn_ms = 0;
			ev |= TERRAIN_EV_TURN_END;
		}
	} else {
		t->turn_ms = 0;
	}
	return ev;
}

/* The car has just left the floor: drop any hump in progress. */
static void leave_floor(terrain_t *t, terrain_lift_t how)
{
	t->lift = how;
	t->state = TERRAIN_FLAT;
	t->timer_ms = 0;
	t->hump_ms = 0;
	t->up_ms = 0;
	t->lean_ms = 0;
	t->over_ms = 0;
	t->down_ms = 0;
	t->level_ms = 0;
}

/* Lifted or overturned?  One step per reading (in g). */
static unsigned int lift_step(terrain_t *t, float x_g, float y_g, float z_g)
{
	float const	g_len = root(t->grav_x*t->grav_x + t->grav_y*t->grav_y +
				     t->grav_z*t->grav_z);
	float		along = t->up_ref_g;
	unsigned int	ev = 0U;

	/* How far the car leans, from an average quick enough to catch a lift */
	t->quick_x += QUICK_SMOOTH * (x_g - t->quick_x);
	t->quick_y += QUICK_SMOOTH * (y_g - t->quick_y);
	t->quick_z += QUICK_SMOOTH * (z_g - t->quick_z);
	t->lean_deg = angle_between(t->quick_x, t->quick_y, t->quick_z,
				    t->flat_x, t->flat_y, t->flat_z);

	/* Speed straight up.  Only the part of the reading that lies along
	   gravity is used: driving, braking and sideways shaking push across
	   gravity, so they add nothing.  A still car reads up_ref_g; more
	   than that means it is being pushed upward.  Adding the extra up
	   over time gives speed; shaking cancels itself out. */
	if (g_len > 0.1f) {
		along = (x_g*t->grav_x + y_g*t->grav_y + z_g*t->grav_z) / g_len;
	}
	/* A hand lifts with a steady push of well under 0.3 g; a single
	   hard bump is a spike far bigger but only one reading long, so
	   each reading's share is capped and one bump can't look like a lift. */
	{
		float push = (along - t->up_ref_g) / t->up_ref_g;

		if (push > UP_STEP_MAX_G) {
			push = UP_STEP_MAX_G;
		}
		if (push < -UP_STEP_MAX_G) {
			push = -UP_STEP_MAX_G;
		}
		t->up_speed += push * G_MS2 * READ_S;
	}
	t->up_speed -= UP_FADE * t->up_speed;
	if (t->up_speed > t->up_speed_max) {
		t->up_speed_max = t->up_speed;
	}

	/* On its side or upside down: checked on the floor and in the hand */
	if (t->lift != TERRAIN_OVERTURNED) {
		if (t->lean_deg > OVERTURN_DEG) {
			t->over_ms += TERRAIN_READ_MS;
			if (t->over_ms >= OVERTURN_MS) {
				leave_floor(t, TERRAIN_OVERTURNED);
				return TERRAIN_EV_OVERTURNED;
			}
		} else {
			t->over_ms = 0;
		}
	}

	if (t->lift == TERRAIN_ON_FLOOR) {
		t->up_ref_g += UP_REF_SMOOTH * (along - t->up_ref_g);

		t->up_ms   = (t->up_speed > LIFT_UP_SPEED) ? (t->up_ms + TERRAIN_READ_MS) : 0;
		t->lean_ms = (t->lean_deg > LIFT_LEAN_DEG) ? (t->lean_ms + TERRAIN_READ_MS) : 0;
		if ((t->up_ms >= LIFT_UP_MS) || (t->lean_ms >= LIFT_LEAN_MS)) {
			leave_floor(t, TERRAIN_LIFTED);
			ev |= TERRAIN_EV_LIFTED;
		}
	} else {
		/* Off the floor: back down once it is level and still for a
		   while.  Wherever it was put down is the new flat.
		   Second way back: level for longer, even while shaking.  That
		   is a car put down and started at once (driving shakes, but
		   a driving car stays level); without it the whole next run
		   would count no humps. */
		t->level_ms = (t->lean_deg < PUT_DOWN_LEVEL_DEG) ? (t->level_ms + TERRAIN_READ_MS) : 0;
		t->down_ms  = ((t->lean_deg < PUT_DOWN_DEG) && (t->shake_g < PUT_DOWN_SHAKE))
			      ? (t->down_ms + TERRAIN_READ_MS) : 0;
		{
			if ((t->down_ms >= PUT_DOWN_MS) || (t->level_ms >= PUT_DOWN_LEVEL_MS)) {
				t->lift = TERRAIN_ON_FLOOR;
				t->flat_x = t->grav_x;
				t->flat_y = t->grav_y;
				t->flat_z = t->grav_z;
				t->up_ref_g = (g_len > 0.5f) ? g_len : t->up_ref_g;
				t->up_speed = 0.0f;
				t->down_ms = 0;
				t->level_ms = 0;
				ev |= TERRAIN_EV_PUT_DOWN;
			}
		}
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
	ev |= lift_step(t, x_g, y_g, z_g);
	ev |= turn_step(t);
	if (t->jolt_g > KNOCK_JOLT) {
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
		/* Humps only count while the car is on the floor */
		if (t->lift == TERRAIN_ON_FLOOR) {
			ev |= hump_step(t);
		}
	}
	return ev;
}

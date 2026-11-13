/*
 *	terrain_logic.h
 *	Subsystem 4 -- hump detector, lifted / overturned detector, and
 *	moving / stationary / turning from the IMU and the wheel speeds.
 *	Private to subsystem 4.
 *	Pure C: no kernel calls and no hardware, so it can be tested on a laptop.
 */

#ifndef TERRAIN_LOGIC_H
#define TERRAIN_LOGIC_H

#define TERRAIN_READ_MS		10	/* terrain_step() is called every 10 ms */

typedef enum { TERRAIN_FLAT, TERRAIN_CLIMBING, TERRAIN_DESCENDING } terrain_state_t;

/* What happened during one terrain_step() (bits, so several can be set) */
#define TERRAIN_EV_CLIMBING	(1U << 0)
#define TERRAIN_EV_CLIMB_AGAIN	(1U << 1)
#define TERRAIN_EV_DESCENDING	(1U << 2)
#define TERRAIN_EV_HUMP_DONE	(1U << 3)
#define TERRAIN_EV_NOT_A_HUMP	(1U << 4)
#define TERRAIN_EV_TIMED_OUT	(1U << 5)
#define TERRAIN_EV_LIFTED	(1U << 6)	/* picked up off the floor */
#define TERRAIN_EV_STARTED	(1U << 7)	/* stationary -> moving */
#define TERRAIN_EV_STOPPED	(1U << 8)	/* moving -> stationary */
#define TERRAIN_EV_OVERTURNED	(1U << 9)	/* on its side or upside down */
#define TERRAIN_EV_PUT_DOWN	(1U << 10)	/* back on the floor, level and still */
#define TERRAIN_EV_TURN_START	(1U << 11)	/* the wheels say the car is turning */
#define TERRAIN_EV_TURN_END	(1U << 12)	/* ... and that it has stopped turning */

/* Is the car on the floor?  Once it leaves TERRAIN_ON_FLOOR it stays that
   way until it has been level and still for a while (TERRAIN_EV_PUT_DOWN). */
typedef enum { TERRAIN_ON_FLOOR, TERRAIN_LIFTED, TERRAIN_OVERTURNED } terrain_lift_t;

typedef struct {
	terrain_state_t	state;
	terrain_lift_t	lift;
	float		grav_x, grav_y, grav_z;	/* averaged gravity direction */
	float		flat_x, flat_y, flat_z;	/* gravity when flat on this floor */
	float		tilt_deg;	/* tilt away from flat */
	float		jolt_g;		/* how far this reading is from the average */
	float		shake_g;	/* averaged jolt: how much the car is shaking */
	int		moving;		/* 1 = moving: shaking OR wheels turning */
	int		shaking;	/* 1 = shaking like a moving car (IMU) */
	int		shake_ms;	/* how long the shake has disagreed with "shaking" */
	int		wheels_ok;	/* 1 = wheel speeds are being supplied */
	int		wheels_seen;	/* bit 0 / 1: left / right wheel has turned at least once */
	float		wheel_l, wheel_r; /* wheel speeds, mm/s, + = forward */
	int		rolling;	/* 1 = a wheel is turning (encoders) */
	int		roll_ms;	/* how long both wheels have been stopped */
	float		turn_dps;	/* turn rate from the wheels, deg/s, + = to the left */
	int		turning;	/* 1 = turning */
	int		turn_ms;	/* how long the turn rate has disagreed with "turning" */
	int		humps;		/* humps counted since the start */
	float		hump_peak_deg;	/* highest tilt in the current hump */
	float		height_mm;	/* height of the last finished hump */
	float		best_mm;	/* highest hump of this run */
	int		timer_ms;	/* how long a condition has lasted */
	int		hump_ms;	/* how long the current hump has lasted */
	int		blank_ms;	/* time left to ignore after a hard knock */
	float		quick_x, quick_y, quick_z; /* faster gravity average, for lift */
	float		lean_deg;	/* quick tilt away from flat: lift / overturn */
	float		up_ref_g;	/* usual reading along gravity (about 0.9 g) */
	float		up_speed;	/* speed straight up, m/s (fades in about 1 s) */
	float		up_speed_max;	/* highest up_speed since the caller cleared it */
	int		up_ms;		/* how long up_speed has been above the limit */
	int		lean_ms;	/* how long lean_deg has been above the lift limit */
	int		over_ms;	/* how long lean_deg has been above the overturn limit */
	int		down_ms;	/* how long a lifted car has been level and still */
	int		level_ms;	/* how long a lifted car has been level (still or not) */
	int		spin_ms;	/* how long one wheel has spun free while tipped */
} terrain_t;

/* Start from the calibration reading (in g), taken flat and still. */
void		terrain_init(terrain_t *t, float x_g, float y_g, float z_g);

/* Optional, call before terrain_step(): the latest wheel speeds in mm/s
   (+ = forward) from the wheel encoders.  ok = 0 means "no wheel data":
   the detector then works from the IMU alone, as it did before. */
void		terrain_wheels(terrain_t *t, int ok, float left_mms, float right_mms);

/* Feed one reading (in g).  Returns the TERRAIN_EV_ bits. */
unsigned int	terrain_step(terrain_t *t, float x_g, float y_g, float z_g);

/* Forget the highest hump: the start of a new run. */
void		terrain_new_run(terrain_t *t);

#endif /* TERRAIN_LOGIC_H */

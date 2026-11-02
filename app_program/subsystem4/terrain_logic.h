/*
 *	terrain_logic.h
 *	Subsystem 4 -- hump and collision detector.  Private to subsystem 4.
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
#define TERRAIN_EV_COLLISION	(1U << 6)
#define TERRAIN_EV_STARTED	(1U << 7)	/* stationary -> moving */
#define TERRAIN_EV_STOPPED	(1U << 8)	/* moving -> stationary */

typedef struct {
	terrain_state_t	state;
	float		grav_x, grav_y, grav_z;	/* averaged gravity direction */
	float		flat_x, flat_y, flat_z;	/* gravity when flat on this floor */
	float		tilt_deg;	/* tilt away from flat */
	float		jolt_g;		/* how far this reading is from the average */
	float		shake_g;	/* averaged jolt: how much the car is shaking */
	int		moving;		/* 1 = shaking like a moving car, 0 = stationary */
	int		shake_ms;	/* how long the shake has disagreed with "moving" */
	float		hump_peak_deg;	/* highest tilt in the current hump */
	float		height_mm;	/* height of the last finished hump */
	float		best_mm;	/* highest hump of this run */
	int		timer_ms;	/* how long a condition has lasted */
	int		hump_ms;	/* how long the current hump has lasted */
	int		blank_ms;	/* time left to ignore after a collision */
} terrain_t;

/* Start from the calibration reading (in g), taken flat and still. */
void		terrain_init(terrain_t *t, float x_g, float y_g, float z_g);

/* Feed one reading (in g).  Returns the TERRAIN_EV_ bits. */
unsigned int	terrain_step(terrain_t *t, float x_g, float y_g, float z_g);

/* Forget the highest hump: the start of a new run. */
void		terrain_new_run(terrain_t *t);

#endif /* TERRAIN_LOGIC_H */

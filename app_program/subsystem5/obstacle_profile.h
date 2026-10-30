/*
 *	obstacle_profile.h
 *	Subsystem 5 -- turn a servo scan into an obstacle profile and a plan.
 *
 *	Pure C, integer only: no kernel calls and no floating point. The
 *	template does not link libm, and the Cortex-M0+ has no FPU (Lab 6),
 *	so sin() comes from a 91-entry Q15 lookup table instead.
 *
 *	Geometry: angle 90 deg = straight ahead. A reading d at angle a is at
 *	  lateral y = d * sin(a - 90)   (positive = car's left,
 *	                                 see OA_SERVO_LEFT_IS_HIGH)
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>
#include "obstacle_config.h"
#include "obstacle_msg.h"

#define OA_MAX_SCAN_POINTS \
	((((OA_SERVO_MAX_DEG) - (OA_SERVO_MIN_DEG)) / (OA_FINE_STEP_DEG)) + 1U)

/*
 * One sweep at evenly spaced angles: reading i was taken at
 * start_deg + i * step_deg. One start + step instead of one angle per
 * reading saves OA_MAX_SCAN_POINTS bytes (Change Requirement 1).
 * step_deg must be > 0 so the angles ascend.
 */
typedef struct
{
	uint8_t	dist_cm[OA_MAX_SCAN_POINTS];
	uint8_t	start_deg;
	uint8_t	step_deg;
	uint8_t	count;
} oa_scan_t;

typedef struct
{
	bool	b_found;
	uint8_t	closest_cm;		/* 255 = unknown.                    */
	uint8_t	closest_angle_deg;
	uint8_t	width_cm;		/* 255 = unknown.                    */
	uint8_t	left_clear_cm;		/* Lateral free gap, 255 = open,
					   0 = blocked or edge not seen.     */
	uint8_t	right_clear_cm;
	int16_t	left_edge_cm;		/* Lateral position of the edges.    */
	int16_t	right_edge_cm;
} oa_profile_t;

/* Angle of reading i. */
uint8_t		scan_angle(oa_scan_t const *p_scan, uint8_t i);

/* sin(deg) in Q15 for -90..+90 deg (clamped). */
int32_t		oa_sin_q15(int32_t deg);

void		profile_reset(oa_profile_t *p_prof);

/* From a coarse scan, choose the fine-scan window. False if nothing hit. */
bool		profile_fine_window(oa_scan_t const *p_coarse,
				    uint8_t *p_start_deg, uint8_t *p_end_deg);

/* Build the profile from a (fine) scan. */
void		profile_build(oa_scan_t const *p_scan, oa_profile_t *p_prof);

/* Pick a plan. *p_sidestep_cm = lateral move (255 unless LEFT/RIGHT). */
obs_plan_t	profile_plan(oa_profile_t const *p_prof, uint8_t retries,
			     uint8_t *p_sidestep_cm);

#endif /* OBSTACLE_PROFILE_H */

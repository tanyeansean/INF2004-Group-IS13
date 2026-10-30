/*
 *	obstacle_profile.c
 *	Subsystem 5 -- profiling (closest point, width, side clearance) and
 *	planning. Integer-only fixed point.
 *
 *	Units: lateral positions are int32_t in Q8 centimetres (cm * 256),
 *	sines are Q15. d (<= 254) * sin (<= 32767) < 2^23, so no overflow.
 *	Only the RESULTS are narrowed to 8-bit fields (Change Requirement 1:
 *	wide maths inside, packed data outside).
 */

#include "obstacle_profile.h"

#define Q8_ONE		(256)
#define Q15_TO_Q8	(128)		/* Q15 -> Q8: divide by 2^7.          */
#define NO_GAP		(INT32_MAX)

/* sin(0..90 deg) * 32767, one entry per degree. */
static int16_t const k_sin_q15[91] =
{
	    0,   572,  1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,
	 5690,  6252,  6813,  7371,  7927,  8481,  9032,  9580, 10126, 10668,
	11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886,
	16383, 16876, 17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621,
	21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964, 24351, 24730,
	25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087,
	28377, 28659, 28932, 29196, 29451, 29697, 29934, 30162, 30381, 30591,
	30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
	32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762,
	32767
};

int32_t oa_sin_q15(int32_t deg)
{
	if (deg > 90) {
		deg = 90;
	} else if (deg < -90) {
		deg = -90;
	} else {
		/* In range. */
	}
	return (deg >= 0) ? (int32_t)k_sin_q15[deg] : -(int32_t)k_sin_q15[-deg];
}

uint8_t scan_angle(oa_scan_t const *p_scan, uint8_t i)
{
	/* Widen before multiplying so the sum cannot wrap in 8 bits (Lab 4). */
	return (uint8_t)((uint16_t)p_scan->start_deg
			 + ((uint16_t)i * (uint16_t)p_scan->step_deg));
}

static int32_t lateral_q8(uint8_t angle_deg, uint8_t dist_cm)
{
	int32_t offset_deg = (int32_t)angle_deg - (int32_t)OA_SERVO_CENTRE_DEG;

#if (OA_SERVO_LEFT_IS_HIGH == 0)
	offset_deg = -offset_deg;
#endif
	/* Division, not >>: right-shifting a negative value is
	   implementation-defined (Barr-C). */
	return ((int32_t)dist_cm * oa_sin_q15(offset_deg)) / Q15_TO_Q8;
}

static bool is_hit(uint8_t dist_cm)
{
	return (dist_cm <= OA_OBSTACLE_RANGE_CM);	/* 255 is never a hit. */
}

/* Q8 cm -> 8-bit cm field, rounded, clamped to 0..254. */
static uint8_t q8_to_cm_field(int32_t value_q8)
{
	int32_t cm;

	if (value_q8 <= 0) {
		return 0U;
	}
	cm = (value_q8 + (Q8_ONE / 2)) / Q8_ONE;
	return (cm >= (int32_t)OA_DIST_MAX_CM) ? (uint8_t)OA_DIST_MAX_CM : (uint8_t)cm;
}

/* Q8 cm -> int16 cm, rounded towards nearest. */
static int16_t q8_to_cm(int32_t value_q8)
{
	return (int16_t)((value_q8 >= 0) ? ((value_q8 + (Q8_ONE / 2)) / Q8_ONE)
					 : -((-value_q8 + (Q8_ONE / 2)) / Q8_ONE));
}

static uint8_t cm_to_field(int32_t cm)
{
	if (cm <= 0) {
		return 0U;
	}
	return (cm >= (int32_t)OA_DIST_MAX_CM) ? (uint8_t)OA_DIST_MAX_CM : (uint8_t)cm;
}

static uint8_t abs_diff_u8(uint8_t a, uint8_t b)
{
	return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

void profile_reset(oa_profile_t *p_prof)
{
	p_prof->b_found           = false;
	p_prof->closest_cm        = OA_DIST_OUT_OF_RANGE;
	p_prof->closest_angle_deg = OA_SERVO_CENTRE_DEG;
	p_prof->width_cm          = OA_DIST_OUT_OF_RANGE;
	p_prof->left_clear_cm     = OA_DIST_OUT_OF_RANGE;
	p_prof->right_clear_cm    = OA_DIST_OUT_OF_RANGE;
	p_prof->left_edge_cm      = 0;
	p_prof->right_edge_cm     = 0;
}

bool profile_fine_window(oa_scan_t const *p_coarse,
			 uint8_t *p_start_deg, uint8_t *p_end_deg)
{
	bool		b_found = false;
	uint16_t	lo = OA_SERVO_MAX_DEG;
	uint16_t	hi = OA_SERVO_MIN_DEG;
	uint16_t	a;
	uint8_t		i;

	for (i = 0U; i < p_coarse->count; i++) {
		if (is_hit(p_coarse->dist_cm[i])) {
			a = scan_angle(p_coarse, i);
			b_found = true;
			lo = (a < lo) ? a : lo;
			hi = (a > hi) ? a : hi;
		}
	}

	if (!b_found) {
		return false;
	}

	*p_start_deg = (lo > (OA_SERVO_MIN_DEG + OA_FINE_MARGIN_DEG))
		     ? (uint8_t)(lo - OA_FINE_MARGIN_DEG) : (uint8_t)OA_SERVO_MIN_DEG;
	*p_end_deg   = ((hi + OA_FINE_MARGIN_DEG) < OA_SERVO_MAX_DEG)
		     ? (uint8_t)(hi + OA_FINE_MARGIN_DEG) : (uint8_t)OA_SERVO_MAX_DEG;

	return true;
}

void profile_build(oa_scan_t const *p_scan, oa_profile_t *p_prof)
{
	uint8_t	i;
	uint8_t	closest = 0U;
	uint8_t	lo;
	uint8_t	hi;
	int32_t	y;
	int32_t	y_left    = INT32_MIN;
	int32_t	y_right   = INT32_MAX;
	int32_t	width;
	int32_t	gap_left  = NO_GAP;
	int32_t	gap_right = NO_GAP;

	profile_reset(p_prof);

	/* 1. Closest hit. */
	for (i = 0U; i < p_scan->count; i++) {
		if (is_hit(p_scan->dist_cm[i]) &&
		    ((!p_prof->b_found) || (p_scan->dist_cm[i] < p_scan->dist_cm[closest]))) {
			closest = i;
			p_prof->b_found = true;
		}
	}

	if (!p_prof->b_found) {
		return;
	}

	/* 2. Grow a cluster of neighbouring hits with no big distance jump. */
	lo = closest;
	hi = closest;
	while ((lo > 0U) && is_hit(p_scan->dist_cm[lo - 1U]) &&
	       (abs_diff_u8(p_scan->dist_cm[lo - 1U], p_scan->dist_cm[lo]) <= OA_CLUSTER_JUMP_CM)) {
		lo--;
	}
	while (((hi + 1U) < p_scan->count) && is_hit(p_scan->dist_cm[hi + 1U]) &&
	       (abs_diff_u8(p_scan->dist_cm[hi + 1U], p_scan->dist_cm[hi]) <= OA_CLUSTER_JUMP_CM)) {
		hi++;
	}

	/* 3. Lateral extent of the cluster. */
	for (i = lo; i <= hi; i++) {
		y = lateral_q8(scan_angle(p_scan, i), p_scan->dist_cm[i]);
		y_left  = (y > y_left)  ? y : y_left;
		y_right = (y < y_right) ? y : y_right;
	}

	/*
	 * 4. Width. The beam is ~15 deg wide, so edges are seen early and the
	 *    raw width is too big by about 2 * d * tan(half beam). TUNE on A3.
	 */
	width = (y_left - y_right)
	      - ((2 * (int32_t)p_scan->dist_cm[closest] * OA_TAN_HALF_BEAM_Q15) / Q15_TO_Q8);

	/* 5. Side clearance: nearest thing outside the cluster on each side. */
	for (i = 0U; i < p_scan->count; i++) {
		if (((i >= lo) && (i <= hi)) || (p_scan->dist_cm[i] == OA_DIST_OUT_OF_RANGE)) {
			continue;
		}

		y = lateral_q8(scan_angle(p_scan, i), p_scan->dist_cm[i]);

		if ((y > y_left) && ((y - y_left) < gap_left)) {
			gap_left = y - y_left;
		} else if ((y < y_right) && ((y_right - y) < gap_right)) {
			gap_right = y_right - y;
		} else {
			/* Behind the obstacle: ignore. */
		}
	}

	/*
	 * 6. If the obstacle runs off the end of the scan, its edge was never
	 *    seen, so that side is UNKNOWN: treat it as blocked, never open.
	 *    (Found by the host "wall" test.) Index 0 = lowest angle.
	 */
	if (0U == lo) {
#if (OA_SERVO_LEFT_IS_HIGH != 0)
		gap_right = 0;
#else
		gap_left = 0;
#endif
	}
	if ((p_scan->count - 1U) == hi) {
#if (OA_SERVO_LEFT_IS_HIGH != 0)
		gap_left = 0;
#else
		gap_right = 0;
#endif
	}

	p_prof->closest_cm        = p_scan->dist_cm[closest];
	p_prof->closest_angle_deg = scan_angle(p_scan, closest);
	p_prof->width_cm          = q8_to_cm_field(width);
	p_prof->left_clear_cm     = (NO_GAP == gap_left)  ? (uint8_t)OA_DIST_OUT_OF_RANGE
							  : q8_to_cm_field(gap_left);
	p_prof->right_clear_cm    = (NO_GAP == gap_right) ? (uint8_t)OA_DIST_OUT_OF_RANGE
							  : q8_to_cm_field(gap_right);
	p_prof->left_edge_cm      = q8_to_cm(y_left);
	p_prof->right_edge_cm     = q8_to_cm(y_right);
}

obs_plan_t profile_plan(oa_profile_t const *p_prof, uint8_t retries,
			uint8_t *p_sidestep_cm)
{
	uint16_t const	need_cm = OA_CAR_WIDTH_CM + (2U * OA_SIDE_MARGIN_CM);
	int32_t const	half_cm = (int32_t)((OA_CAR_WIDTH_CM / 2U) + OA_SIDE_MARGIN_CM);
	bool		b_left_ok;
	bool		b_right_ok;
	int32_t		step_left;
	int32_t		step_right;

	*p_sidestep_cm = OA_DIST_OUT_OF_RANGE;

	if (!p_prof->b_found) {
		return OBS_PLAN_CONTINUE;
	}

	b_left_ok  = (p_prof->left_clear_cm  >= need_cm);	/* 255 = open. */
	b_right_ok = (p_prof->right_clear_cm >= need_cm);

	/* Lateral move so the car's near side clears the obstacle edge. */
	step_left  = ( (int32_t)p_prof->left_edge_cm)  + half_cm;
	step_right = (-(int32_t)p_prof->right_edge_cm) + half_cm;

	/* Brief: minimise deviation, so prefer the smaller sidestep. */
	if (b_left_ok && ((!b_right_ok) || (step_left <= step_right))) {
		*p_sidestep_cm = cm_to_field(step_left);
		return OBS_PLAN_LEFT;
	}
	if (b_right_ok) {
		*p_sidestep_cm = cm_to_field(step_right);
		return OBS_PLAN_RIGHT;
	}

	return (retries < OA_MAX_RETRIES) ? OBS_PLAN_REVERSE : OBS_PLAN_STOP;
}

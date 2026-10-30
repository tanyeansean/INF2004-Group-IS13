/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5. Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md. IDs: S5-U01, ...
 *
 *	These check the pure logic ON THE RP2040 (same compiler, -O2, no FPU),
 *	which is the Week 13 "correctness" evidence on the real target. The
 *	same cases also run on a laptop: testing/subsystem5/host/.
 *	Hardware behaviour (echo, servo, stop time) is manual: see TEST_LOG.md.
 */

#include "subsystem5.h"
#include "obstacle_profile.h"
#include "../test_report.h"

#if APP_TEST

LOCAL void make_scan(oa_scan_t *p_s, UB start, UB step, UB end, UB (*p_dist)(UB))
{
	UH a;

	p_s->start_deg = start;
	p_s->step_deg  = step;
	p_s->count     = 0U;
	for (a = start; (a <= end) && (p_s->count < OA_MAX_SCAN_POINTS); a += step) {
		p_s->dist_cm[p_s->count] = p_dist((UB)a);
		p_s->count++;
	}
}

LOCAL UB box(UB a)     { return ((a >= 70U) && (a <= 100U)) ? 25U : ((a <= 50U) ? 45U : 255U); }
LOCAL UB nothing(UB a) { (void)a; return 255U; }
LOCAL UB wall(UB a)    { (void)a; return 20U; }

LOCAL BOOL msg_roundtrip_all(void)
{
	obstacle_msg_t	in;
	obstacle_msg_t	out;
	uint8_t		buf[OBSTACLE_MSG_SIZE];
	UW		t, p, s, v;

	for (t = 0U; t <= 3U; t++) {
		for (p = 0U; p <= (UW)OBS_PLAN_REVERSE; p++) {
			for (s = 0U; s <= 7U; s++) {
				for (v = 0U; v <= 255U; v += 51U) {
					in.type = (obs_msg_type_t)t; in.plan = (obs_plan_t)p;
					in.state = (uint8_t)s;
					in.closest_cm = in.width_cm = in.left_clear_cm =
					in.right_clear_cm = in.sidestep_cm = (uint8_t)v;
					obstacle_msg_pack(&in, buf);
					if (!obstacle_msg_unpack(buf, &out) ||
					    (out.type != in.type) || (out.plan != in.plan) ||
					    (out.state != in.state) || (out.closest_cm != v) ||
					    (out.sidestep_cm != v)) {
						return FALSE;
					}
				}
			}
		}
	}
	return TRUE;
}

EXPORT void subsystem5_test(void)
{
	oa_scan_t	s;
	oa_profile_t	p;
	uint8_t		step;
	uint8_t		buf[OBSTACLE_MSG_SIZE] = {0};
	obstacle_msg_t	out;
	obs_plan_t	plan;
	s5_state_t	st;

	TEST_INIT();

	TEST_CHECK("S5-U01", msg_roundtrip_all(), "obstacle_q pack/unpack round-trips");

	buf[0] = (uint8_t)(5U << 2);	/* plan code 5 is undefined */
	TEST_CHECK("S5-U02", !obstacle_msg_unpack(buf, &out), "undefined plan code rejected");

	make_scan(&s, 40U, 5U, 140U, box);
	profile_build(&s, &p);
	plan = profile_plan(&p, 0U, &step);
	TEST_CHECK("S5-U03", p.b_found && (p.closest_cm == 25U) && (plan == OBS_PLAN_LEFT),
		   "box right of centre -> LEFT");
	tm_printf((UB*)"        box: width=%d L=%d R=%d sidestep=%d\n",
		  p.width_cm, p.left_clear_cm, p.right_clear_cm, step);

	make_scan(&s, 15U, 5U, 165U, nothing);
	profile_build(&s, &p);
	TEST_CHECK("S5-U04", (!p.b_found) && (profile_plan(&p, 0U, &step) == OBS_PLAN_CONTINUE),
		   "empty scan -> CONTINUE");

	make_scan(&s, 15U, 5U, 165U, wall);
	profile_build(&s, &p);
	TEST_CHECK("S5-U05", (profile_plan(&p, 0U, &step) == OBS_PLAN_REVERSE) &&
			     (profile_plan(&p, OA_MAX_RETRIES, &step) == OBS_PLAN_STOP),
		   "wall wider than scan -> REVERSE, then STOP");

	TEST_CHECK("S5-U06", (oa_sin_q15(90) == 32767) && (oa_sin_q15(-30) == -16383) &&
			     (oa_sin_q15(0) == 0), "sine table sign and range");

	st = s5_get_state();
	TEST_CHECK("S5-U07", (UW)st <= (UW)S5_STATE_FAULT, "obstacle_task state is valid");

	TEST_CHECK("S5-U08", s5_dropped_msgs() == 0U, "no obstacle_q messages dropped");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/**
 * @file    test_obstacle.c
 * @brief   Host tests for obstacle_msg and obstacle_profile (no hardware needed).
 *
 * Build and run on a laptop, from this folder:
 *   S=../../../app_program/subsystem5
 *   gcc -std=c11 -Wall -Wextra -I$S test_obstacle.c $S/obstacle_msg.c \
 *       $S/obstacle_profile.c -o test_obstacle && ./test_obstacle
 * (Only the two pure modules are built: they have no kernel calls.)
 * The same cases run on the Pico in subsystem5_test.c (make TEST=1).
 *
 * Week 13 evidence:
 *   - Item 4 (correctness): every packed field round-trips over its full range.
 *   - Item 3 (message size): prints packed vs unpacked sizes.
 */
#include <stdio.h>
#include <string.h>
#include "obstacle_msg.h"
#include "obstacle_profile.h"

static int g_fails = 0;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            g_fails++;                                    \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);   \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

/* "Before" layout: the same fields with no packing (enums are int-sized). */
typedef struct
{
    int      type;
    int      plan;
    int      state;
    uint32_t closest_cm;
    uint32_t width_cm;
    uint32_t left_clear_cm;
    uint32_t right_clear_cm;
    uint32_t sidestep_cm;
} legacy_obstacle_msg_t;

/* "Before" scan buffer: one angle stored per reading. */
typedef struct
{
    uint8_t angle_deg[OA_MAX_SCAN_POINTS];
    uint8_t dist_cm[OA_MAX_SCAN_POINTS];
    uint8_t count;
} legacy_scan_t;

static void
test_roundtrip_header (void)
{
    unsigned type, plan, state, n = 0U;

    for (type = 0U; type <= 3U; type++)
    {
        for (plan = 0U; plan <= (unsigned)OBS_PLAN_REVERSE; plan++)
        {
            for (state = 0U; state <= 7U; state++)
            {
                obstacle_msg_t in = { (obs_msg_type_t)type, (obs_plan_t)plan,
                                      (uint8_t)state, 1U, 2U, 3U, 4U, 5U };
                obstacle_msg_t out;
                uint8_t buf[OBSTACLE_MSG_SIZE];

                obstacle_msg_pack(&in, buf);
                CHECK(obstacle_msg_unpack(buf, &out), "valid header rejected");
                CHECK((out.type == in.type) && (out.plan == in.plan) &&
                      (out.state == in.state),
                      "header t=%u p=%u s=%u did not round-trip", type, plan, state);
                n++;
            }
        }
    }
    printf("header round-trip: %u combinations\n", n);
}

static void
test_roundtrip_bytes (void)
{
    unsigned v;

    for (v = 0U; v <= 255U; v++)
    {
        obstacle_msg_t in = { OBS_MSG_PLAN, OBS_PLAN_LEFT, 6U,
                              (uint8_t)v, (uint8_t)v, (uint8_t)v, (uint8_t)v, (uint8_t)v };
        obstacle_msg_t out;
        uint8_t buf[OBSTACLE_MSG_SIZE];

        obstacle_msg_pack(&in, buf);
        (void)obstacle_msg_unpack(buf, &out);
        CHECK((out.closest_cm == v) && (out.width_cm == v) && (out.left_clear_cm == v) &&
              (out.right_clear_cm == v) && (out.sidestep_cm == v),
              "byte value %u did not round-trip", v);
    }
    printf("byte fields round-trip: 0..255\n");
}

static void
test_reject_bad_plan (void)
{
    uint8_t buf[OBSTACLE_MSG_SIZE] = { 0 };
    obstacle_msg_t out;
    unsigned plan;

    for (plan = (unsigned)OBS_PLAN_REVERSE + 1U; plan <= 7U; plan++)
    {
        buf[0] = (uint8_t)(plan << 2);
        CHECK(!obstacle_msg_unpack(buf, &out), "undefined plan %u accepted", plan);
    }
    printf("undefined plan codes 5..7 rejected\n");
}

/* Build a scan from a function of angle. */
static void
make_scan (oa_scan_t * p_s, uint8_t start, uint8_t step, uint8_t end,
           uint8_t (*p_dist)(uint8_t))
{
    uint16_t a;

    memset(p_s, 0, sizeof(*p_s));
    p_s->start_deg = start;
    p_s->step_deg  = step;
    for (a = start; (a <= end) && (p_s->count < OA_MAX_SCAN_POINTS); a += step)
    {
        p_s->dist_cm[p_s->count] = p_dist((uint8_t)a);
        p_s->count++;
    }
}

static uint8_t box_right_of_centre (uint8_t a) { return ((a >= 70U) && (a <= 100U)) ? 25U : ((a <= 50U) ? 45U : 255U); }
static uint8_t nothing (uint8_t a)             { (void)a; return 255U; }
static uint8_t wall (uint8_t a)                { (void)a; return 20U; }

static void
test_profile_scenarios (void)
{
    oa_scan_t    s;
    oa_profile_t p;
    uint8_t      step;
    obs_plan_t   plan;

    make_scan(&s, 40U, 5U, 140U, box_right_of_centre);
    profile_build(&s, &p);
    plan = profile_plan(&p, 0U, &step);
    CHECK(p.b_found && (p.closest_cm == 25U), "box not found at 25 cm");
    CHECK(plan == OBS_PLAN_LEFT, "box: expected LEFT, got %d", (int)plan);
    printf("box scenario: closest=%u width=%u L=%u R=%u plan=%d sidestep=%u\n",
           p.closest_cm, p.width_cm, p.left_clear_cm, p.right_clear_cm, (int)plan, step);

    make_scan(&s, 15U, 5U, 165U, nothing);
    profile_build(&s, &p);
    CHECK(!p.b_found, "empty scan reported an obstacle");
    CHECK(profile_plan(&p, 0U, &step) == OBS_PLAN_CONTINUE, "empty scan: expected CONTINUE");

    /* Obstacle wider than the scan: neither side may be reported open. */
    make_scan(&s, 15U, 5U, 165U, wall);
    profile_build(&s, &p);
    plan = profile_plan(&p, 0U, &step);
    CHECK((p.left_clear_cm == 0U) && (p.right_clear_cm == 0U),
          "wall: sides should be blocked, got L=%u R=%u", p.left_clear_cm, p.right_clear_cm);
    CHECK(plan == OBS_PLAN_REVERSE, "wall: expected REVERSE, got %d", (int)plan);
    p.left_clear_cm = 5U;
    p.right_clear_cm = 5U;
    CHECK(profile_plan(&p, 0U, &step) == OBS_PLAN_REVERSE, "blocked: expected REVERSE");
    CHECK(profile_plan(&p, OA_MAX_RETRIES, &step) == OBS_PLAN_STOP, "blocked, retries out: expected STOP");

    /* Angle maths must not wrap near the top of the 8-bit range. */
    s.start_deg = 160U;
    s.step_deg  = 5U;
    CHECK(scan_angle(&s, 1U) == 165U, "scan_angle wrapped");
    printf("profile scenarios: box, empty, wall, blocked, angle overflow\n");
}

static void
print_sizes (void)
{
    printf("\n--- Week 13 item 3: message / buffer sizes (bytes) ---\n");
    printf("%-28s %8s %8s\n", "item", "before", "after");
    printf("%-28s %8zu %8u\n", "obstacle_q message",
           sizeof(legacy_obstacle_msg_t), (unsigned)OBSTACLE_MSG_SIZE);
    printf("%-28s %8zu %8zu\n", "scan buffer (oa_scan_t)",
           sizeof(legacy_scan_t), sizeof(oa_scan_t));
    printf("(host sizes; recheck with the ARM build - see README)\n");
}

int
main (void)
{
    test_roundtrip_header();
    test_roundtrip_bytes();
    test_reject_bad_plan();
    test_profile_scenarios();
    print_sizes();

    printf("\n%s (%d failure%s)\n", (0 == g_fails) ? "PASS" : "FAIL",
           g_fails, (1 == g_fails) ? "" : "s");
    return (0 == g_fails) ? 0 : 1;
}

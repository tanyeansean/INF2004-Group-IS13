/* Subsystem 1 PC test of the Safety Monitor attendance logic (safety_logic.c).
 * Simulates the kernel's 10 ms tick: every wait is rounded up to the next
 * tick plus up to one tick of jitter, like tk_dly_tsk() on the car. */
#include <stdio.h>
#include <stdlib.h>
#include "safety_logic.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } else { printf("ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static unsigned tick_wait(unsigned ms) {           /* tk_dly_tsk(ms) with a 10 ms tick */
  unsigned t = ((ms + 9u) / 10u) * 10u;
  return t + (unsigned)(rand() % 2) * 10u;          /* plus 0 or 1 tick */
}
/* task periods as on the car: Motor 20, Mission 20, Line 5, Barcode 5,
   Terrain ~20, Obstacle up to ~100 (50 ms chunks + 30 ms ping), Telemetry 10 */
static const unsigned period[SM_TASKS] = {20, 20, 5, 5, 20, 60, 10};

typedef struct { unsigned next[SM_TASKS]; unsigned stop_at[SM_TASKS]; } sim_t;

/* Run for dur ms; returns first time a task became hung (or 0), sets *who. */
static unsigned run(sm_state_t *sm, sim_t *sim, unsigned *now, unsigned dur, unsigned *who, unsigned *max_gap) {
  unsigned pending = 0, end = *now + dur, next_check = *now + tick_wait(20), last[SM_TASKS];
  for (unsigned i = 0; i < SM_TASKS; i++) last[i] = *now;
  *who = 0;
  for (; *now < end; (*now)++) {
    for (unsigned i = 0; i < SM_TASKS; i++)
      if (*now >= sim->next[i] && *now < sim->stop_at[i]) {
        pending |= 1u << i;
        if (*now - last[i] > max_gap[i]) max_gap[i] = *now - last[i];
        last[i] = *now;
        sim->next[i] = *now + tick_wait(period[i]) + ((i == 5) ? (unsigned)(rand() % 30) : 0u);
      }
    if (*now >= next_check && (*now % 10u) == 0u) {   /* monitor wakes on a tick */
      unsigned newly = sm_check(sm, pending, *now, NULL);
      pending = 0;
      next_check = *now + tick_wait(20);
      if (newly && !*who) { *who = newly; return *now; }
    }
  }
  return 0;
}

int main(void) {
  sm_state_t sm; sim_t sim; unsigned now = 0, who, t, gap[SM_TASKS] = {0};
  srand(7);
  for (unsigned i = 0; i < SM_TASKS; i++) { sim.next[i] = 0; sim.stop_at[i] = ~0u; }
  sm_init(&sm);
  sm_seen(&sm, HB_ALL_TASKS, 0); sm_restart(&sm, 0);

  t = run(&sm, &sim, &now, 600000, &who, gap);       /* 10 minutes, everyone healthy */
  CHECK(t == 0, "10 min with every task healthy: no false hung (worst gaps: motor %u, line %u, obstacle %u, telemetry %u ms)",
        gap[0], gap[2], gap[5], gap[6]);

  sim.stop_at[0] = now + 5;                          /* Motor Control freezes */
  unsigned frozen = now + 5;
  t = run(&sm, &sim, &now, 1000, &who, gap);
  CHECK(who == HB_MOTOR_CONTROL && t - frozen <= SM_LIMIT_MS + 70,
        "Motor Control freezes: reported hung %u ms later (limit %u + up to 2 checks of 30 ms + 1 tick)", t - frozen, SM_LIMIT_MS);
  sim.stop_at[0] = ~0u; sim.next[0] = now;
  unsigned rec = 0; (void)run(&sm, &sim, &now, 100, &who, gap);
  (void)sm_check(&sm, HB_MOTOR_CONTROL, now, &rec);
  CHECK(sm.hung == 0, "Motor Control beats again: no longer hung");

  sim.stop_at[5] = now + 5; frozen = now + 5;        /* Obstacle Avoid freezes */
  t = run(&sm, &sim, &now, 1000, &who, gap);
  CHECK(who == HB_OBSTACLE_AVOID && t - frozen <= SM_LIMIT_OBSTACLE_MS + 70,
        "Obstacle Avoid freezes: reported %u ms later (limit %u)", t - frozen, SM_LIMIT_OBSTACLE_MS);

  sm_init(&sm); sm_seen(&sm, HB_ALL_TASKS & ~HB_OBSTACLE_AVOID, now); sm_restart(&sm, now);
  CHECK(sm_check(&sm, HB_ALL_TASKS & ~HB_OBSTACLE_AVOID, now + 5000, NULL) == 0,
        "a task that never beat (S5 not built in) is never reported");
  CHECK(sm_check(&sm, 0, now + 5000 + SM_LIMIT_MS + 10, NULL) == (HB_ALL_TASKS & ~HB_OBSTACLE_AVOID),
        "every watched task stops at once: all reported in one check");
  CHECK(sm_check(&sm, 0, now + 9000, NULL) == 0, "a hung task is reported once, not every check");
  CHECK(sm_first(HB_TELEMETRY) == 7 && sm_first(HB_MOTOR_CONTROL | HB_LINE_FOLLOW) == 1,
        "hung task number for telemetry: Motor = 1, Telemetry = 7");
  printf(fails ? "%d FAILED\n" : "ALL PASSED\n", fails);
  return fails != 0;
}

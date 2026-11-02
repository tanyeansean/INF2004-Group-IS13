/*
 *	safety_logic.h
 *	Subsystem 1 -- Safety Monitor attendance logic (Buddy 1, Aurelius).
 *
 *	No kernel calls in here, so the same code runs in the car
 *	(task_safety.c) and in the PC test (README.md, "Tests").
 *
 *	Each task sets its own HB_* bit (hb_bits.h) on flg_hb every time it
 *	wakes.  The Safety Monitor reads and clears all the bits every check
 *	and passes them to sm_check().  A task counts as hung once it has gone
 *	longer than its limit without a beat.  A task is only watched after its
 *	first beat, so a subsystem that is not built in (APP_WITH_S5 = 0) or not
 *	started yet is never reported as hung.
 */

#ifndef SAFETY_LOGIC_H
#define SAFETY_LOGIC_H

#include <stdint.h>
#include "hb_bits.h"

#define SM_TASKS		(7U)	/* HB_* bits 0-6 */

/*
 * Hung limits: time without a beat.  The kernel tick is 10 ms
 * (config/config.h, CNF_TIMER_PERIOD), so a task's "20 ms" wait really
 * takes 20-30 ms and every time read here moves in 10 ms steps.  The G9
 * table's 40 ms (2 x 20 ms, written for a 1 ms tick) therefore becomes
 * 60 ms here.  Obstacle Avoid waits in 50 ms chunks plus a 30 ms ping.
 * PROVISIONAL until the worst gaps are measured on the car.
 */
#define SM_LIMIT_MS		(60U)
#define SM_LIMIT_OBSTACLE_MS	(150U)

typedef struct {
	uint32_t	last_ms[SM_TASKS];	/* time of each task's last beat */
	uint32_t	armed;			/* HB_* bits: has beaten at least once */
	uint32_t	hung;			/* HB_* bits: past its limit now */
	uint32_t	ever_hung;		/* HB_* bits: hung at least once */
} sm_state_t;

void		sm_init(sm_state_t *p_s);

/* Limit in ms for one task (task = bit number 0-6). */
uint32_t	sm_limit_ms(uint32_t task);

/* Record beats only (no hung check): used before monitoring starts. */
void		sm_seen(sm_state_t *p_s, uint32_t seen, uint32_t now_ms);

/* Restart every task's clock: called once, when monitoring starts, so the
   boot and calibration time never counts as a gap. */
void		sm_restart(sm_state_t *p_s, uint32_t now_ms);

/*
 * One attendance check.  seen = HB_* bits set since the last check.
 * Returns the tasks that became hung in this check.  *p_recovered (if not
 * NULL) gets the hung tasks that beat again in this check.
 */
uint32_t	sm_check(sm_state_t *p_s, uint32_t seen, uint32_t now_ms,
			 uint32_t *p_recovered);

/* 1-7 = bit number + 1 of the lowest set bit in mask, 0 = mask empty.
   Used for the 3-bit hung-task field in the telemetry fault mask. */
uint32_t	sm_first(uint32_t mask);

#endif /* SAFETY_LOGIC_H */

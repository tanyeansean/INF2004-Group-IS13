/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle Avoid: ultrasonic + servo scanning, obstacle
 *	profiling, avoidance planning. Student ID: 2500177 (Joe Teo Wei Xuan).
 *
 *	Public interface: only what OTHER subsystems may use.
 *	Sensing only: S5 never drives the motors. It reports on obstacle_q and
 *	Mission Control (S2) steers, then answers through flg_avoid.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "obstacle_msg.h"	/* obstacle_q message layout (IF table) */
#include "s5_links.h"		/* S5_STANDALONE and the objects S5 uses */

/* Priority band 20-21 (TEAM.md section 4). 1 is the highest. */
#define S5_PRI_OBSTACLE		20
#define S5_PRI_BENCH		21	/* stand-in Mission Control, bench only */

/* ---------------------------------------------------------------------
 *  S5 -> S2: obstacle_q. Owned by the RECEIVER (S2, Mission Control),
 *  per the template rule; see s5_links.h for how S5 finds it.
 *  Each message is exactly OBSTACLE_MSG_SIZE bytes (obstacle_msg.h).
 *
 *  S2 -> S5: flg_avoid. Owned by S5 (the receiver). Mission Control
 *  sets these bits with tk_set_flg(); S5 waits on and clears them.
 * --------------------------------------------------------------------- */
IMPORT ID	flg_avoid;

#define AVOID_EVT_MOTORS_STOPPED	(1U << 0)	/* car stationary (after stop
							   or a finished reverse)   */
#define AVOID_EVT_LINE_REACQUIRED	(1U << 1)	/* line found after bypass  */
#define AVOID_EVT_RESUME		(1U << 2)	/* fault cleared, resume    */
#define AVOID_EVT_MC_READY		(1U << 3)	/* Mission Control is now
							   draining obstacle_q: S5
							   may start pinging       */
#define AVOID_EVT_ABORT			(1U << 4)	/* Mission Control stopped
							   the car mid-avoid (STOP,
							   GP20, give-up): S5 goes
							   back to MONITOR, no fault */

/* S5 states, 3 bits (also byte 0 bits 5-7 of every obstacle_q message). */
typedef enum
{
	S5_STATE_MONITOR = 0,	/* ping straight ahead every 60 ms   */
	S5_STATE_WAIT_STOP,	/* asked to stop/reverse, waiting    */
	S5_STATE_COARSE_SCAN,	/* 6 angles, 65-115 deg (10 deg steps) */
	S5_STATE_FINE_SCAN,	/* 5 deg steps around the hits       */
	S5_STATE_PROFILE,	/* closest point, width, clearances  */
	S5_STATE_PLAN,		/* LEFT/RIGHT/REVERSE/STOP/CONTINUE  */
	S5_STATE_BYPASS,	/* watch ahead, wait LINE_REACQUIRED */
	S5_STATE_FAULT		/* latched until AVOID_EVT_RESUME    */
} s5_state_t;

/* Current state, for telemetry. Safe from any task, either core. */
IMPORT s5_state_t	s5_get_state(void);

/* obstacle_q messages that could not be queued (queue full). */
IMPORT UW		s5_dropped_msgs(void);

/* ---------------------------------------------------------------------
 *  For the team's shared GPIO interrupt dispatcher (IO_IRQ_BANK0).
 *  Call this with GP5's 4-bit event nibble (bit2 EDGE_LOW, bit3
 *  EDGE_HIGH) AFTER clearing those edges in INTR. It only timestamps.
 *  See README "GPIO interrupt" before building with S5_OWN_GPIO_IRQ=0.
 * --------------------------------------------------------------------- */
IMPORT void		drv_hcsr04_edge(UW events);

/* Private to subsystem 5 */
IMPORT void		obstacle_task(INT stacd, void *exinf);
#if S5_STANDALONE
IMPORT void		s5_bench_task(INT stacd, void *exinf);
#endif

#endif /* SUBSYSTEM5_H */

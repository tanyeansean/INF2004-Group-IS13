/*
 *	s5_links.h
 *	Subsystem 5 -- the kernel objects S5 USES but other subsystems OWN,
 *	and the bit positions inside them. Private to S5.
 *
 *	S5_STANDALONE = 1 (default until S1/S2 publish their objects):
 *	  subsystem5_init() creates stand-ins for the heartbeat flag, the
 *	  fault flag and obstacle_q, and starts s5_bench_task, a fake Mission
 *	  Control that answers S5 and prints every message. S5 can then be
 *	  bench-tested on its own.
 *	S5_STANDALONE = 0: use the real objects from S1 (Safety Monitor) and
 *	  S2 (Mission Control). Change only the names in the #else branch.
 *
 *	Assumed mapping (Tribunal 1 buddies -> template folders):
 *	  Buddy 1 Aurelius -> subsystem1 (Safety Monitor, telemetry)
 *	  Buddy 2 Sean     -> subsystem2 (Mission Control, motors, fault table)
 */

#ifndef S5_LINKS_H
#define S5_LINKS_H

#include "../app.h"

#ifndef S5_STANDALONE
#define S5_STANDALONE		1
#endif

/* 1: S5 registers IO_IRQ_BANK0 itself (bench only: it then OWNS the
   GPIO interrupt for every pin). 0: the team dispatcher calls
   drv_hcsr04_edge(). Only ONE tk_def_int() for IRQ 13 may exist. */
#ifndef S5_OWN_GPIO_IRQ
#define S5_OWN_GPIO_IRQ		S5_STANDALONE
#endif

#if S5_STANDALONE
IMPORT ID	s5_stub_hb_flg;
IMPORT ID	s5_stub_fault_flg;
IMPORT ID	s5_stub_obstacle_mbf;
#define S5_HB_FLG		(s5_stub_hb_flg)
#define S5_FAULT_FLG		(s5_stub_fault_flg)
#define S5_OBSTACLE_MBF		(s5_stub_obstacle_mbf)
#else
#include "../subsystem1/subsystem1.h"	/* TODO: real heartbeat flag ID */
#include "../subsystem2/subsystem2.h"	/* TODO: real fault flag, obstacle_q */
#define S5_HB_FLG		(flg_heartbeat)
#define S5_FAULT_FLG		(flg_faults)
#define S5_OBSTACLE_MBF		(mbf_obstacle)
#endif

/* Heartbeat bit: HB_OBSTACLE_AVOID in Aurelius's integrated_car/comms/hb_bits.h
   (bit 4 there is Terrain Monitor). Fault bits: still to agree with Sean. */
#define S5_HB_BIT		(1U << 5)
#define S5_FAULT_NO_ECHO	(1U << 8)	/* HC-SR04 never raises ECHO      */
#define S5_FAULT_NO_ROUTE	(1U << 9)	/* both sides blocked, retries out */
#define S5_FAULT_TIMEOUT	(1U << 10)	/* stop ack / line reacquire late */

/* obstacle_q sizing for the stand-in (the real one is S2's choice). */
#define S5_OBSTACLE_MBF_MSGS	(4U)

#endif /* S5_LINKS_H */

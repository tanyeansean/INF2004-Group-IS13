/*
 *	s5_links.h
 *	Subsystem 5 -- the kernel objects S5 USES but other subsystems OWN,
 *	and the bit positions inside them. Private to S5.
 *
 *	S5_STANDALONE = 0 (default, integrated car):
 *	  heartbeat  -> flg_hb (S1), bit HB_OBSTACLE_AVOID (hb_bits.h)
 *	  obstacle_q -> mbf_obstacle (S2, Mission Control: the receiver owns it)
 *	  faults     -> flg_faults (S2), bits FAULT_BIT_US_* (shared_types.h)
 *	  Replies come back on flg_avoid (owned by S5, subsystem5.h).
 *	S5_STANDALONE = 1 (bench only, S5 alone on a board):
 *	  subsystem5_init() creates stand-ins for all three and starts
 *	  s5_bench_task, a fake Mission Control that answers and prints.
 *	  Do NOT use it in the car build: the real Mission Control would then
 *	  never hear about obstacles.
 */

#ifndef S5_LINKS_H
#define S5_LINKS_H

#include "../app.h"

#ifndef S5_STANDALONE
#define S5_STANDALONE		0
#endif

/* 1: S5 registers IO_IRQ_BANK0 itself. ONLY for a board running S5 and
   nothing else that uses GPIO interrupts. 0 (default): subsystem 3's
   handler is the shared GPIO dispatcher and calls drv_hcsr04_edge() for
   GP5. Only ONE tk_def_int() for IRQ 13 may exist. */
#ifndef S5_OWN_GPIO_IRQ
#define S5_OWN_GPIO_IRQ		0
#endif

#if S5_STANDALONE
IMPORT ID	s5_stub_hb_flg;
IMPORT ID	s5_stub_fault_flg;
IMPORT ID	s5_stub_obstacle_mbf;
#define S5_HB_FLG		(s5_stub_hb_flg)
#define S5_FAULT_FLG		(s5_stub_fault_flg)
#define S5_OBSTACLE_MBF		(s5_stub_obstacle_mbf)
#define S5_HB_BIT		(1U << 5)
#define S5_FAULT_NO_ECHO	(1U << 8)
#define S5_FAULT_NO_ROUTE	(1U << 9)
#define S5_FAULT_TIMEOUT	(1U << 10)
#else
#include "../subsystem1/subsystem1.h"	/* flg_hb, HB_OBSTACLE_AVOID     */
#include "../subsystem2/subsystem2.h"	/* mbf_obstacle, flg_faults, FAULT_BIT_* */
#define S5_HB_FLG		(flg_hb)
#define S5_FAULT_FLG		(flg_faults)
#define S5_OBSTACLE_MBF		(mbf_obstacle)
#define S5_HB_BIT		(HB_OBSTACLE_AVOID)
#define S5_FAULT_NO_ECHO	(FAULT_BIT_US_NO_ECHO)	/* bit 8  */
#define S5_FAULT_NO_ROUTE	(FAULT_BIT_NO_SAFE_RTE)	/* bit 9  */
#define S5_FAULT_TIMEOUT	(FAULT_BIT_US_TIMEOUT)	/* bit 10 */
#endif

/* obstacle_q sizing for the stand-in (the real one is S2's choice). */
#define S5_OBSTACLE_MBF_MSGS	(4U)

#endif /* S5_LINKS_H */

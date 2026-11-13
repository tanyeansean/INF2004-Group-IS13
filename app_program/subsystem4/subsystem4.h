/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU / terrain (Buddy 4, Lim Jin Hin).  Student ID: 2500928.
 *
 *	Reads the GY-511 on I2C1 (GP2, GP3) and detects humps, hump height, and
 *	the car being lifted or overturned (which replaced collision detection
 *	on 13 Nov).  Reports with HB_TERRAIN_MONITOR on flg_hb, fault bits on
 *	flg_faults, and terrain_q (below).  When the car is lifted or overturned
 *	it also sends CMD_ESTOP on mbf_cmd (IF-04) so the motors stop.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MAIN	18		/* priority band 18-19 (TEAM.md) */

/* terrain_q (S4 -> S2): one terrain_msg_t (2 bytes) whenever the terrain
   state changes, a hump finishes, or the car is lifted, overturned or put
   back down.  Bit 6 ("collision" in shared_types.h) now means: the car is
   off the floor.  Read it with
   tk_rcv_mbf() and unpack with terrain_get_*() (../subsystem2/shared_types.h).
   The highest-hump field is in mm. */
IMPORT ID	mbf_terrain;

/* ---------------------------------------------------------------------
 *  Latest terrain report: ONE 32-bit word, every field only as wide as it
 *  needs (Change Requirement 1).  Any task on either core may read it with
 *  s4_report(): reading one aligned 32-bit word is a single step, so no
 *  lock is needed and a reader can never see half an update.  Only
 *  terrain_task writes it, every 10 ms.
 *
 *    bits  0-6   tilt away from flat, 0.5 deg steps (0 .. 63.5 deg)
 *    bits  7-14  turn rate from the wheels, signed, 2 deg/s steps
 *                (-254 .. +254 deg/s, + = turning left; 0 = no wheel data)
 *    bits 15-17  latest motion event (imu_event_t; 5 = lifted / overturned)
 *    bits 18-19  terrain state (0 flat, 1 climbing, 2 descending)
 *    bits 20-21  0 on the floor, 1 lifted, 2 overturned
 *    bits 22-28  height of the last finished hump, mm (0 .. 127).  The
 *                highest of the run is in terrain_q (below)
 *    bits 29-31  hump counter: goes up by 1 for every hump, wraps at 8
 *
 *  For Telemetry (it must be the task that calls telem_*()): each poll,
 *    UW r = s4_report();
 *    telem_set_imu(s4_report_tilt_deg(r), (float)s4_report_turn_dps(r),
 *                  (imu_event_t)s4_report_motion(r));
 *    and when s4_report_humps(r) changes:
 *    telem_event_hump((float)s4_report_last_hump_mm(r) / 10.0f);
 * --------------------------------------------------------------------- */
IMPORT volatile UW	s4_report_word;

static inline UW    s4_report(void)               { return s4_report_word; }
static inline float s4_report_tilt_deg(UW r)      { return (float)(r & 0x7FU) * 0.5f; }
static inline INT   s4_report_turn_dps(UW r)      { return (INT)(signed char)((r >> 7) & 0xFFU) * 2; }
static inline UINT  s4_report_motion(UW r)        { return (UINT)((r >> 15) & 0x07U); }
static inline UINT  s4_report_terrain(UW r)       { return (UINT)((r >> 18) & 0x03U); }
static inline UINT  s4_report_lift(UW r)          { return (UINT)((r >> 20) & 0x03U); }
static inline UINT  s4_report_last_hump_mm(UW r)  { return (UINT)((r >> 22) & 0x7FU); }
static inline UINT  s4_report_humps(UW r)         { return (UINT)((r >> 29) & 0x07U); }

/* Start of a new run: forget the highest hump.  Any task may call it (for
   example on START or on the laptop's reset_peak command); terrain_task
   does the work on its next reading. */
IMPORT void	s4_new_run(void);

/* Private to subsystem 4 */
IMPORT void	terrain_task(INT stacd, void *exinf);
IMPORT ER	terrain_objects_init(void);

#endif /* SUBSYSTEM4_H */

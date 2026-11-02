/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU / terrain (Buddy 4, Lim Jin Hin).  Student ID: 2500928.
 *
 *	Reads the GY-511 on I2C1 (GP2, GP3) and detects humps, hump height and
 *	collisions.  Reports with HB_TERRAIN_MONITOR on flg_hb, fault bits on
 *	flg_faults, and terrain_q (below).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MAIN	18		/* priority band 18-19 (TEAM.md) */

/* terrain_q (S4 -> S2): one terrain_msg_t (2 bytes) whenever the terrain
   state changes, a hump finishes or the car is hit.  Read it with
   tk_rcv_mbf() and unpack with terrain_get_*() (../subsystem2/shared_types.h).
   The highest-hump field is in mm. */
IMPORT ID	mbf_terrain;

/* Private to subsystem 4 */
IMPORT void	terrain_task(INT stacd, void *exinf);
IMPORT ER	terrain_objects_init(void);

#endif /* SUBSYSTEM4_H */

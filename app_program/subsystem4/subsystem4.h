/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU / terrain (Buddy 4, Lim Jin Hin).  Student ID: 2500928.
 *
 *	Reads the GY-511 on I2C1 (GP2, GP3) and detects humps, hump height and
 *	collisions.  It only prints its results so far: the links to the rest of
 *	the car (heartbeat, fault bits, terrain_q) are the next step.  The
 *	terrain_q message layout is drafted in ../subsystem2/shared_types.h.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MAIN	18		/* priority band 18-19 (TEAM.md) */

/* Private to subsystem 4 */
IMPORT void	terrain_task(INT stacd, void *exinf);
IMPORT ER	terrain_objects_init(void);

#endif /* SUBSYSTEM4_H */

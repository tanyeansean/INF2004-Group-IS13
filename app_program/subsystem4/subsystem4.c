/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU / terrain (Buddy 4, Lim Jin Hin): objects and tasks.
 *
 *	subsystem4_init():  kernel objects only (called before ANY task starts)
 *	subsystem4_start(): tasks
 */

#include "subsystem4.h"

EXPORT ER subsystem4_init(void)
{
	return terrain_objects_init();
}

EXPORT ER subsystem4_start(void)
{
	/* Core 0, away from the steering tasks on core 1: reading the sensor
	   uses a little processor time every 10 ms. */
	ID tid = app_start_task(terrain_task, S4_PRI_MAIN, TP_PRC1, "terrain_task");

	return (tid < E_OK) ? tid : E_OK;
}

/*
 *	subsystem4.c
 *	Subsystem 4 -- <name> (Buddy 4, Jin Hin: IMU / terrain).  EMPTY TEMPLATE.
 *
 *	The template's example monitor that used to live here depended on the
 *	example code in subsystems 1 and 3, which were replaced by the car's
 *	tasks; it is in git history (commit 7f3cd6f) if needed.
 *
 *	subsystem4_init():  create this subsystem's kernel objects.  Do not
 *	                    start tasks here.
 *	subsystem4_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S4_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem4.h"

EXPORT ER subsystem4_init(void)
{
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	return E_OK;
}

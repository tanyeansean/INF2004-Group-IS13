/*
 *	drv_imu.h
 *	Subsystem 4 -- GY-511 sensor hardware.  Private to subsystem 4.
 *	Only terrain_task calls these (TEAM.md: one owning task per peripheral).
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "../app.h"

#define IMU_LSB_PER_G	8192.0f		/* raw number for 1 g at +/-4 g range */

IMPORT ER	drv_imu_init(void);
IMPORT ER	drv_imu_read(H *p_x, H *p_y, H *p_z);

#endif /* DRV_IMU_H */

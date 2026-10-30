/*
 *	drv_servo.h
 *	Subsystem 5 -- scan servo on GP12 (PWM slice 6 A), 50 Hz, 1 us steps.
 *	Call only from obstacle_task.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "../app.h"

IMPORT void	drv_servo_init(void);
IMPORT void	drv_servo_set_angle(UB angle_deg);	/* clamped */
IMPORT UB	drv_servo_get_angle(void);

/* Estimated time to move between two angles, including settle time. */
IMPORT UW	drv_servo_travel_ms(UB from_deg, UB to_deg);

#endif /* DRV_SERVO_H */

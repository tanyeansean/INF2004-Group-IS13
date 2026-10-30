/*
 *	task_motor.c
 *	Subsystem 2 -- Motor Control task: the ONLY task that writes the motor
 *	PWM.  Applies each wheel command from Mission Control as soon as it
 *	arrives (same latency as integrated_car, where the main loop called
 *	motor_set() directly), and wakes at least every 20 ms for its heartbeat.
 *	Open-loop stand-in: Sean's 20 ms per-wheel PID goes here later.
 */

#include "subsystem2.h"
#include "../subsystem1/subsystem1.h"	/* flg_hb */

#define S2_MOTOR_WAIT_MS	20

EXPORT void motor_task(INT stacd, void *exinf)
{
	UW	m;
	INT	n;

	(void)stacd; (void)exinf;

	for (;;) {
		n = tk_rcv_mbf(mbf_motor, &m, S2_MOTOR_WAIT_MS);
		if (n == (INT)sizeof(m)) {
			drv_motor_set(s2_wheel_get(m, 0U), s2_wheel_get(m, S2_WHEEL_BITS));
		}
		(void)tk_set_flg(flg_hb, HB_MOTOR_CONTROL);
	}
}

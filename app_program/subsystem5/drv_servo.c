/*
 *	drv_servo.c
 *	Subsystem 5 -- scan servo, using the board library (bsp/libbsp.h).
 *
 *	PWM maths (Lab 4): clk_sys = 125 MHz. Divider 125 -> 1 MHz counter,
 *	wrap 19999 -> 20000 counts (the +1) -> 50 Hz. One count = 1 us, so
 *	the compare level IS the pulse width in microseconds.
 *	libbsp has no divider call, so the DIV register is written directly:
 *	integer part in bits 11:4, fraction in bits 3:0.
 *
 *	Note: libbsp's pwm_set_cc() read-modify-writes the CC register shared
 *	by channels A and B of slice 6. If someone uses GP13 (slice 6 B) from
 *	a task on the OTHER core, agree a lock or move the servo.
 */

#include <sys/machine.h>
#include "drv_servo.h"
#include "obstacle_config.h"

#define SERVO_CLKDIV_INT	(125U)		/* 125 MHz / 125 = 1 MHz */
#define SERVO_WRAP		(19999U)	/* 20 ms period          */
#define SERVO_FULL_DEG		(180U)
#define SERVO_SLICE		((OA_PIN_SERVO >> 1) & 0x07U)
#define S5_PWM_BASE		(0x40050000U)
#define S5_PWM_DIV(slice)	(S5_PWM_BASE + (0x14U * (slice)) + 0x04U)

LOCAL UB s_angle_deg = OA_SERVO_CENTRE_DEG;	/* obstacle_task only */

EXPORT void drv_servo_init(void)
{
	pwm_set_pin(OA_PIN_SERVO);
	out_w(S5_PWM_DIV(SERVO_SLICE), SERVO_CLKDIV_INT << 4);
	pwm_set_wrap(OA_PIN_SERVO, SERVO_WRAP);
	drv_servo_set_angle(OA_SERVO_CENTRE_DEG);
	pwm_set_enabled(OA_PIN_SERVO, TRUE);
}

EXPORT void drv_servo_set_angle(UB angle_deg)
{
	UW pulse_us;

	if (angle_deg < OA_SERVO_MIN_DEG) {
		angle_deg = OA_SERVO_MIN_DEG;
	} else if (angle_deg > OA_SERVO_MAX_DEG) {
		angle_deg = OA_SERVO_MAX_DEG;
	} else {
		/* In range. */
	}

	pulse_us = OA_SERVO_PULSE_MIN_US
		 + (((UW)angle_deg * (OA_SERVO_PULSE_MAX_US - OA_SERVO_PULSE_MIN_US))
		    / SERVO_FULL_DEG);

	pwm_set_cc(OA_PIN_SERVO, pulse_us);
	s_angle_deg = angle_deg;
}

EXPORT UB drv_servo_get_angle(void)
{
	return s_angle_deg;
}

EXPORT UW drv_servo_travel_ms(UB from_deg, UB to_deg)
{
	UW const delta = (from_deg > to_deg) ? (UW)(from_deg - to_deg)
					     : (UW)(to_deg - from_deg);

	return (delta * OA_SERVO_MS_PER_DEG) + OA_SERVO_SETTLE_MS;
}

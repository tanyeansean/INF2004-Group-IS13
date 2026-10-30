/*
 *	drv_motor.c
 *	Subsystem 2 -- Robo Pico dual DC motor PWM (port of integrated_car/motor.c).
 *
 *	  M1: GP8 (M1A), GP9 (M1B)   -> PWM slice 4 A/B
 *	  M2: GP10 (M2A), GP11 (M2B) -> PWM slice 5 A/B
 *	Left wheel = M1 (MOTOR_LEFT_IS_M1), verified on the car 26 Oct.
 *	Speed -255..+255: + = PWM on A, B low; - = A low, PWM on B; 0 = coast.
 *	10 kHz: 125 MHz / (256 x 48.8125) = 10.0 kHz (motor.c used the same).
 *	Only motor_task calls drv_motor_set() (one owner of the PWM).
 */

#include <sys/machine.h>
#include "subsystem2.h"

#ifndef MOTOR_LEFT_IS_M1
#define MOTOR_LEFT_IS_M1	1
#endif
#ifndef MOTOR_INVERT_LEFT
#define MOTOR_INVERT_LEFT	0
#endif
#ifndef MOTOR_INVERT_RIGHT
#define MOTOR_INVERT_RIGHT	0
#endif

#define PIN_M1A		8U
#define PIN_M1B		9U
#define PIN_M2A		10U
#define PIN_M2B		11U

#define PWM_WRAP	255U
#define PWM_DIV_INT	48U		/* 125e6 / (256 * 10e3) = 48.83 */
#define PWM_DIV_FRAC	13U		/* .8125 = 13/16                */

#define S2_RESETS_RESET	(0x4000C000U)
#define S2_RESETS_DONE	(0x4000C008U)
#define S2_RESET_PWM	(1U << 14)
#define REG_ALIAS_CLR	(0x3000U)
#define S2_PWM_DIV(ch)	(0x40050000U + 0x04U + (0x14U * (ch)))

LOCAL void slice_init(UINT pin_a, UINT pin_b)
{
	UINT ch = (pin_a >> 1) & 0x07U;

	out_w(S2_PWM_DIV(ch), (PWM_DIV_INT << 4) | PWM_DIV_FRAC);
	(void)pwm_set_wrap(pin_a, PWM_WRAP);
	(void)pwm_set_cc(pin_a, 0U);
	(void)pwm_set_cc(pin_b, 0U);
	(void)pwm_set_pin(pin_a);
	(void)pwm_set_pin(pin_b);
	(void)pwm_set_enabled(pin_a, TRUE);
}

LOCAL H clamp_speed(H s)
{
	if (s > S2_PWM_MAX) {
		return S2_PWM_MAX;
	}
	if (s < -S2_PWM_MAX) {
		return (H)-S2_PWM_MAX;
	}
	return s;
}

LOCAL void drive_channel(UINT pin_a, UINT pin_b, H speed)
{
	speed = clamp_speed(speed);
	if (speed >= 0) {
		(void)pwm_set_cc(pin_a, (UW)speed);
		(void)pwm_set_cc(pin_b, 0U);
	} else {
		(void)pwm_set_cc(pin_a, 0U);
		(void)pwm_set_cc(pin_b, (UW)(-speed));
	}
}

EXPORT void drv_motor_init(void)
{
	/* PWM block out of reset (clear alias, no read-modify-write). */
	if ((in_w(S2_RESETS_RESET) & S2_RESET_PWM) != 0U) {
		out_w(S2_RESETS_RESET + REG_ALIAS_CLR, S2_RESET_PWM);
		while ((in_w(S2_RESETS_DONE) & S2_RESET_PWM) == 0U) {
			;
		}
	}
	slice_init(PIN_M1A, PIN_M1B);
	slice_init(PIN_M2A, PIN_M2B);
}

EXPORT void drv_motor_set(H left, H right)
{
#if MOTOR_INVERT_LEFT
	left = (H)-clamp_speed(left);
#endif
#if MOTOR_INVERT_RIGHT
	right = (H)-clamp_speed(right);
#endif
#if MOTOR_LEFT_IS_M1
	drive_channel(PIN_M1A, PIN_M1B, left);
	drive_channel(PIN_M2A, PIN_M2B, right);
#else
	drive_channel(PIN_M2A, PIN_M2B, left);
	drive_channel(PIN_M1A, PIN_M1B, right);
#endif
}

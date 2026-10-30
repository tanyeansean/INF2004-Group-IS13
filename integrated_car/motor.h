/**
 * @file motor.h
 * @brief Open-loop PWM drive for the Robo Pico's two DC motors.
 *
 * Based on motor_test.c. Robo Pico motor driver pins:
 *   M1: GP8 (M1A), GP9 (M1B)   -> PWM slice 4
 *   M2: GP10 (M2A), GP11 (M2B) -> PWM slice 5
 *
 * Speed is signed, -255..+255 (0 = coast). No encoders / PID here; that is
 * Buddy 2's motor_control.c at full integration.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_MAX_SPEED  255

/* Which Robo Pico channel is the car's LEFT wheel. Swap if the car turns
   the wrong way at a junction. */
#ifndef MOTOR_LEFT_IS_M1
#define MOTOR_LEFT_IS_M1 1
#endif

/* Set to 1 if that wheel runs backwards for a positive speed (or swap its
   two motor wires in the screw terminal). */
#ifndef MOTOR_INVERT_LEFT
#define MOTOR_INVERT_LEFT  0
#endif
#ifndef MOTOR_INVERT_RIGHT
#define MOTOR_INVERT_RIGHT 0
#endif

/** Set up both PWM slices at ~20 kHz, motors stopped. */
void motor_init(void);

/** Drive both wheels. Values are clamped to -255..+255. */
void motor_set(int16_t left, int16_t right);

/** Both wheels coast (both driver inputs low). */
void motor_stop(void);

/** Bench test: drive Robo Pico channel 1 (M1) or 2 (M2) directly, ignoring
    the left/right mapping and invert settings. */
void motor_set_channel(uint8_t channel, int16_t speed);

#endif /* MOTOR_H */

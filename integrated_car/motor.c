/**
 * @file motor.c
 * @brief Robo Pico dual DC motor PWM drive (from motor_test.c).
 *
 * Changes vs motor_test.c:
 *   - both motors (M1 = GP8/9, M2 = GP10/11)
 *   - PWM frequency set to ~10 kHz from the real system clock
 *     (motor_test.c's clkdiv 4 gave ~122 kHz at 125 MHz)
 *   - speed clamped to -255..+255
 */
#include "motor.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define PIN_M1A  8u
#define PIN_M1B  9u
#define PIN_M2A  10u
#define PIN_M2B  11u

#define PWM_WRAP     255u
#define PWM_FREQ_HZ  10000.0f   /* 10 kHz: inside Cytron drivers' 20 kHz max */

static void slice_init(uint pin_a, uint pin_b)
{
    uint slice = pwm_gpio_to_slice_num(pin_a);

    gpio_set_function(pin_a, GPIO_FUNC_PWM);
    gpio_set_function(pin_b, GPIO_FUNC_PWM);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) /
                          ((float)(PWM_WRAP + 1u) * PWM_FREQ_HZ));
    pwm_set_gpio_level(pin_a, 0);
    pwm_set_gpio_level(pin_b, 0);
    pwm_set_enabled(slice, true);
}

static int16_t clamp_speed(int16_t s)
{
    if (s > MOTOR_MAX_SPEED)  { return MOTOR_MAX_SPEED; }
    if (s < -MOTOR_MAX_SPEED) { return (int16_t)-MOTOR_MAX_SPEED; }
    return s;
}

/* Positive: PWM on A, B low. Negative: A low, PWM on B. */
static void drive_channel(uint pin_a, uint pin_b, int16_t speed)
{
    speed = clamp_speed(speed);
    if (speed >= 0) {
        pwm_set_gpio_level(pin_a, (uint16_t)speed);
        pwm_set_gpio_level(pin_b, 0);
    } else {
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, (uint16_t)(-speed));
    }
}

void motor_init(void)
{
    slice_init(PIN_M1A, PIN_M1B);
    slice_init(PIN_M2A, PIN_M2B);
}

void motor_set(int16_t left, int16_t right)
{
#if MOTOR_INVERT_LEFT
    left = (int16_t)-clamp_speed(left);
#endif
#if MOTOR_INVERT_RIGHT
    right = (int16_t)-clamp_speed(right);
#endif
#if MOTOR_LEFT_IS_M1
    drive_channel(PIN_M1A, PIN_M1B, left);
    drive_channel(PIN_M2A, PIN_M2B, right);
#else
    drive_channel(PIN_M2A, PIN_M2B, left);
    drive_channel(PIN_M1A, PIN_M1B, right);
#endif
}

void motor_stop(void)
{
    motor_set(0, 0);
}

void motor_set_channel(uint8_t channel, int16_t speed)
{
    if (channel == 1u) {
        drive_channel(PIN_M1A, PIN_M1B, speed);
    } else if (channel == 2u) {
        drive_channel(PIN_M2A, PIN_M2B, speed);
    }
}

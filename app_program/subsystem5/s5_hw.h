/*
 *	s5_hw.h
 *	Subsystem 5 -- small RP2040 helpers the template does not provide.
 *
 *	The template does NOT link the Pico SDK runtime (no time_us_32(),
 *	no hardware/pwm.h), and the kernel only takes some peripherals out of
 *	reset: PWM only when USE_PTMR=1 (it is 0). These helpers fill the gap.
 */

#ifndef S5_HW_H
#define S5_HW_H

#include "../app.h"

/*
 * Take TIMER and PWM out of reset and start the 1 MHz timer tick, if not
 * already done. Clear-only and idempotent, so it is safe if another
 * subsystem does the same. Belongs in shared start-up code (see README):
 * Sean's motor PWM needs the PWM block too.
 */
IMPORT void	s5_hw_init(void);

/* Free-running 1 MHz microsecond counter (TIMER TIMERAWL). Wraps ~71 min;
   always compare with (now - then), which is wrap-safe in unsigned. */
IMPORT UW	s5_time_us(void);

/* Busy-wait a few microseconds (only for the 10 us trigger pulse). */
IMPORT void	s5_busy_wait_us(UW us);

/* Index of the core running this code: 0 or 1 (SIO CPUID). */
IMPORT UW	s5_core_num(void);

#endif /* S5_HW_H */

/*
 *	s5_hw.c
 *	Subsystem 5 -- RP2040 timer / reset helpers (see s5_hw.h).
 *
 *	Register writes use the RP2040's atomic CLEAR alias (+0x3000), so a
 *	write never read-modify-writes a register the other core may be
 *	changing (Change Requirement 1: no RMW races on shared packed data).
 */

#include <sys/machine.h>
#include "s5_hw.h"

#define REG_ALIAS_CLR		(0x3000U)

#define S5_RESETS_BASE		(0x4000C000U)
#define S5_RESETS_RESET		(S5_RESETS_BASE + 0x0U)
#define S5_RESETS_DONE		(S5_RESETS_BASE + 0x8U)
#define S5_RESET_PWM		(1U << 14)
#define S5_RESET_TIMER		(1U << 21)

#define S5_WDT_TICK		(0x4005802CU)	/* WATCHDOG TICK register */
#define S5_WDT_TICK_ENABLE	(1U << 9)
#define S5_XOSC_MHZ		(12U)		/* clk_ref = 12 MHz XOSC  */

#define S5_TIMER_TIMERAWL	(0x40054028U)
#define S5_SIO_CPUID		(0xD0000000U)

EXPORT void s5_hw_init(void)
{
	UW const need = S5_RESET_PWM | S5_RESET_TIMER;

	/* Release from reset (clear alias: no RMW), then wait until done. */
	if ((in_w(S5_RESETS_RESET) & need) != 0U) {
		out_w(S5_RESETS_RESET + REG_ALIAS_CLR, need);
		while ((in_w(S5_RESETS_DONE) & need) != need) {
			;
		}
	}

	/* The TIMER counts 1 us ticks from the watchdog tick generator.
	   Always set it to 12 cycles (1 us at clk_ref 12 MHz). After a
	   power-on or reset it is already enabled with CYCLES = 0 (reset
	   value 0x200), so the TIMER would run about 43x slow and every
	   echo would read as noise / out of range. Same fix as S3's
	   drv_track_timer_init() in commit 648d108 (Jin, 13 Nov). S5 must
	   not depend on S3 initialising first. */
	out_w(S5_WDT_TICK, S5_WDT_TICK_ENABLE | S5_XOSC_MHZ);
}

EXPORT UW s5_time_us(void)
{
	return in_w(S5_TIMER_TIMERAWL);
}

EXPORT void s5_busy_wait_us(UW us)
{
	UW const start = s5_time_us();

	while ((s5_time_us() - start) < us) {
		;
	}
}

EXPORT UW s5_core_num(void)
{
	return in_w(S5_SIO_CPUID);
}

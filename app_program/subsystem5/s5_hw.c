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
#define S5_WDT_TICK_RUNNING	(1U << 10)
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

	/* The TIMER counts 1 us ticks from the watchdog tick generator. */
	if ((in_w(S5_WDT_TICK) & S5_WDT_TICK_RUNNING) == 0U) {
		out_w(S5_WDT_TICK, S5_WDT_TICK_ENABLE | S5_XOSC_MHZ);
	}
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

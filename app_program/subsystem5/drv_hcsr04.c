/*
 *	drv_hcsr04.c
 *	Subsystem 5 -- HC-SR04P driver. Echo width timed by GPIO edge interrupt.
 *
 *	ISR <-> task sharing (Change Requirement 1, last paragraph):
 *	  - Each shared variable has ONE writer at a time: the task writes the
 *	    flags before triggering, the ISR writes them while armed.
 *	  - Each is one aligned 32-bit word, so a read or write is a single
 *	    access (atomic on the M0+). No read-modify-write on shared data.
 *	  - They are deliberately NOT packed into one byte: packing would make
 *	    every write a read-modify-write shared with the ISR.
 *	  - volatile: the compiler cannot see the hardware call the ISR (Lab 3).
 *	  - The ISR and obstacle_task run on the SAME core (core 0), so they
 *	    never run at the same instant even in the SMP build.
 */

#include <sys/machine.h>
#include "drv_hcsr04.h"
#include "obstacle_config.h"
#include "s5_hw.h"
#include "s5_links.h"
#include "subsystem5.h"

#define ECHO_DONE_BIT		(1U << 0)
#define TRIG_PULSE_US		(10U)

/* IO_BANK0 interrupt registers: 4 bits per GPIO, 8 GPIOs per register. */
#define IO_BANK0		(0x40014000U)
#define IO_INTR(pin)		(IO_BANK0 + 0x0F0U + (4U * ((pin) / 8U)))
#define IO_PROC_INTE(c, pin)	(IO_BANK0 + 0x100U + (0x30U * (c)) + (4U * ((pin) / 8U)))
#define IO_PROC_INTS(c, pin)	(IO_BANK0 + 0x120U + (0x30U * (c)) + (4U * ((pin) / 8U)))
#define IO_SHIFT(pin)		(4U * ((pin) % 8U))
#define IO_EVT_EDGE_LOW		(1U << 2)
#define IO_EVT_EDGE_HIGH	(1U << 3)
#define IO_EVT_EDGES		(IO_EVT_EDGE_LOW | IO_EVT_EDGE_HIGH)
#define REG_ALIAS_SET		(0x2000U)

#define INTNO_IO_BANK0		(13U)	/* RP2040 IO_IRQ_BANK0 */
#define S5_GPIO_INTPRI		(2)

/*
 * Pulses shorter than this are noise, not echoes: the sensor's minimum
 * range is 2 cm = 116 us. Typical cause: the 10 us TRIG pulse coupling
 * into the ECHO wire next to it in the Grove cable, or both edges of a
 * spike seen in one interrupt (width ~0). Without this check they read
 * as "2 cm" and look like an obstacle touching the car.
 */
#define ECHO_MIN_US		(OA_DIST_MIN_VALID_CM * OA_US_PER_CM)	/* 6 cm = 348 us */

#if (OA_DIST_MIN_VALID_CM >= OA_BYPASS_ABORT_CM) || (OA_DIST_MIN_VALID_CM >= OA_STOP_DIST_CM)
#error "OA_DIST_MIN_VALID_CM would hide real obstacles: keep it below the stop/abort distances."
#endif

LOCAL ID		s_echo_flg = 0;
LOCAL volatile UW	s_armed = 0U;		/* task sets, ISR clears   */
LOCAL volatile UW	s_rise_seen = 0U;	/* ISR sets, task reads    */
LOCAL volatile UW	s_rise_us = 0U;		/* ISR only                */
LOCAL volatile UW	s_width_us = 0U;	/* ISR writes, task reads  */
LOCAL volatile UW	s_glitches = 0U;	/* ISR only writer, task reads */
LOCAL volatile UW	s_edges = 0U;		/* ISR only writer: GP5 edges seen */
LOCAL volatile UW	s_short_seen = 0U;	/* task clears before arming, ISR sets */

EXPORT void drv_hcsr04_edge(UW events)
{
	UW const now_us = s5_time_us();
	UW	 width;

	s_edges++;
	if (s_armed == 0U) {
		return;
	}
	if ((events & IO_EVT_EDGE_HIGH) != 0U) {
		s_rise_us   = now_us;
		s_rise_seen = 1U;
	}
	if (((events & IO_EVT_EDGE_LOW) != 0U) && (s_rise_seen != 0U)) {
		width = now_us - s_rise_us;
		if (width < ECHO_MIN_US) {
			/* Noise: forget it and keep waiting for the real echo. */
			s_glitches++;
			s_short_seen = 1U;
			s_rise_seen = 0U;
			return;
		}
		s_width_us = width;
		s_armed    = 0U;
		(void)tk_set_flg(s_echo_flg, ECHO_DONE_BIT);	/* allowed in handlers */
	}
}

EXPORT void drv_hcsr04_diag(UW *p_edges, UW *p_glitches, UW *p_last_width_us)
{
	*p_edges         = s_edges;
	*p_glitches      = s_glitches;
	*p_last_width_us = s_width_us;
}

#if S5_OWN_GPIO_IRQ
/*
 * BENCH ONLY: S5 owns IO_IRQ_BANK0. It serves GP5 only, so do not enable
 * GPIO interrupts on any other pin in this build or they will never be
 * cleared and the interrupt will fire forever.
 */
LOCAL void gpio_bank0_inthdr(UINT intno)
{
	UW const core = s5_core_num();
	UW const ev   = (in_w(IO_PROC_INTS(core, OA_PIN_ECHO)) >> IO_SHIFT(OA_PIN_ECHO)) & 0xFU;

	(void)intno;
	/* Edge bits are write-1-to-clear: clear only ours. */
	out_w(IO_INTR(OA_PIN_ECHO), (ev & IO_EVT_EDGES) << IO_SHIFT(OA_PIN_ECHO));
	drv_hcsr04_edge(ev);
}
#endif

EXPORT ER drv_hcsr04_create(void)
{
	T_CFLG cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WSGL;
	cflg.iflgptn = 0U;
	s_echo_flg = tk_cre_flg(&cflg);
	return (s_echo_flg < E_OK) ? (ER)s_echo_flg : E_OK;
}

EXPORT ER drv_hcsr04_init(void)
{
	UW const	core = s5_core_num();
	ER		er = E_OK;
#if S5_OWN_GPIO_IRQ
	T_DINT		dint = {0};
#endif

	gpio_set_pin(OA_PIN_TRIG, GPIO_MODE_OUT);
	gpio_set_val(OA_PIN_TRIG, 0U);
	gpio_set_pin(OA_PIN_ECHO, GPIO_MODE_IN);

	/* Drop any stale edge before enabling. */
	out_w(IO_INTR(OA_PIN_ECHO), IO_EVT_EDGES << IO_SHIFT(OA_PIN_ECHO));

#if S5_OWN_GPIO_IRQ
	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_bank0_inthdr;
	er = tk_def_int(INTNO_IO_BANK0, &dint);
	if (er < E_OK) {
		return er;
	}
	ClearInt(INTNO_IO_BANK0);
	EnableInt(INTNO_IO_BANK0, S5_GPIO_INTPRI);
#endif

	/* Enable both edges of GP5 for THIS core (atomic SET alias: the INTE
	   register also holds GP0-GP7 bits that other subsystems own). */
	out_w(IO_PROC_INTE(core, OA_PIN_ECHO) + REG_ALIAS_SET,
	      IO_EVT_EDGES << IO_SHIFT(OA_PIN_ECHO));

	return er;
}

EXPORT hcsr04_status_t drv_hcsr04_measure_cm(UB *p_dist_cm)
{
	UINT	flgptn = 0U;
	ER	er;
	UW	cm;

	*p_dist_cm = (UB)OA_DIST_OUT_OF_RANGE;

	if (gpio_get_val(OA_PIN_ECHO) != 0U) {
		return HCSR04_BUSY;
	}

	(void)tk_clr_flg(s_echo_flg, ~ECHO_DONE_BIT);
	s_rise_seen  = 0U;
	s_short_seen = 0U;
	s_armed      = 1U;

	gpio_set_val(OA_PIN_TRIG, 1U);
	s5_busy_wait_us(TRIG_PULSE_US);
	gpio_set_val(OA_PIN_TRIG, 0U);

	er = tk_wai_flg(s_echo_flg, ECHO_DONE_BIT, TWF_ORW | TWF_BITCLR,
			&flgptn, (TMO)OA_ECHO_TIMEOUT_MS);
	s_armed = 0U;

	if (er != E_OK) {
		/* Rose but did not fall in time: beyond our cap (HC-SR04 holds
		   ECHO ~38 ms when nothing is in range). Never rose: fault. */
		if (s_rise_seen != 0U) {
			return HCSR04_OK;
		}
		/* The sensor did answer, but only with a too-short pulse. */
		return (s_short_seen != 0U) ? HCSR04_TOO_SHORT : HCSR04_NO_ECHO;
	}

	cm = (s_width_us + (OA_US_PER_CM / 2U)) / OA_US_PER_CM;	/* rounded */

	if (cm > OA_DIST_MAX_CM) {
		*p_dist_cm = (UB)OA_DIST_OUT_OF_RANGE;
	} else if (cm < OA_DIST_MIN_CM) {
		*p_dist_cm = (UB)OA_DIST_MIN_CM;
	} else {
		*p_dist_cm = (UB)cm;
	}
	return HCSR04_OK;
}

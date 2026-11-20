/*
 *	drv_track.c
 *	Subsystem 3 -- hardware for the IR sensors (Buddy 3).
 *
 *	  Front IR pair  : GP27 = ADC1 (left), GP26 = ADC0 (right), read with
 *	                   the kernel's ADC driver ("adca").  Owner: line task.
 *	  Barcode IR D0  : GP7, both edges, interrupt timestamps only.
 *	                   Owner: barcode task (core 0, same core as the ISR).
 *	  1 us timer     : TIMER TIMERAWL (read-only after start-up).
 *
 *	GPIO interrupt (IO_IRQ_BANK0, IRQ 13): only ONE tk_def_int() may exist
 *	for it.  Subsystem 3 owns it and it is the shared dispatcher: GP7 here,
 *	Joe's echo (GP5, subsystem 5: drv_hcsr04_edge() takes the GP5 event
 *	nibble) and Sean's wheel encoders (GP0/GP1, GP16/GP17, subsystem 2:
 *	drv_encoder_irq() reads and clears its own pins).
 *	Subsystem 5 must then be built with S5_OWN_GPIO_IRQ=0.
 *
 *	ISR <-> task: the ISR only pushes timestamps into the SPSC edge queue
 *	(barcode_decode.h); it is the queue's only producer and the barcode
 *	task its only consumer, both on core 0.
 */

#include <sys/machine.h>
#include "subsystem3.h"
#include "drv_track.h"
#if APP_WITH_S5
#include "../subsystem5/subsystem5.h"	/* drv_hcsr04_edge() for GP5 */
#endif
#include "../subsystem2/drv_encoder.h"	/* drv_encoder_irq(): GP0/1, GP16/17 */

/* ---- RP2040 registers not covered by libbsp ---- */
#define REG_ALIAS_SET		(0x2000U)
#define REG_ALIAS_CLR		(0x3000U)

#define S3_RESETS_RESET		(0x4000C000U)
#define S3_RESETS_DONE		(0x4000C008U)
#define S3_RESET_TIMER		(1U << 21)
#define S3_WDT_TICK		(0x4005802CU)
#define S3_WDT_TICK_ENABLE	(1U << 9)
#define S3_WDT_TICK_RUNNING	(1U << 10)
#define S3_XOSC_MHZ		(12U)
#define S3_TIMERAWL		(0x40054028U)
#define S3_SIO_CPUID		(0xD0000000U)

#define S3_IO_BANK0		(0x40014000U)
#define S3_IO_INTR(pin)		(S3_IO_BANK0 + 0x0F0U + (4U * ((pin) / 8U)))
#define S3_IO_PROC_INTE(c, pin)	(S3_IO_BANK0 + 0x100U + (0x30U * (c)) + (4U * ((pin) / 8U)))
#define S3_IO_PROC_INTS(c, pin)	(S3_IO_BANK0 + 0x120U + (0x30U * (c)) + (4U * ((pin) / 8U)))
#define S3_IO_SHIFT(pin)	(4U * ((pin) % 8U))
#define S3_EVT_EDGE_LOW		(1U << 2)
#define S3_EVT_EDGE_HIGH	(1U << 3)
#define S3_EVT_EDGES		(S3_EVT_EDGE_LOW | S3_EVT_EDGE_HIGH)

#define S3_PADS_BANK0		(0x4001C000U)
#define S3_PAD(pin)		(S3_PADS_BANK0 + 0x04U + (4U * (pin)))
#define S3_PAD_PDE		(1U << 2)
#define S3_PAD_PUE		(1U << 3)
#define S3_PAD_IE		(1U << 6)
#define S3_GPIO_CTRL(pin)	(S3_IO_BANK0 + 0x04U + (8U * (pin)))
#define S3_FUNCSEL_NULL		(31U)

#define S3_INTNO_IO_BANK0	(13U)
#define S3_GPIO_INTPRI		(2)

/* The barcode edge queue: ISR producer, barcode task consumer. */
EXPORT barcode_edge_queue_t	s3_edge_q;

LOCAL ID	s3_adc_dd = 0;

/* ---- time ---- */
EXPORT void drv_track_timer_init(void)
{
	/* Release TIMER from reset (clear alias, no read-modify-write) and
	   start its 1 MHz tick.  Idempotent: other subsystems may do it too. */
	if ((in_w(S3_RESETS_RESET) & S3_RESET_TIMER) != 0U) {
		out_w(S3_RESETS_RESET + REG_ALIAS_CLR, S3_RESET_TIMER);
		while ((in_w(S3_RESETS_DONE) & S3_RESET_TIMER) == 0U) {
			;
		}
	}
	/* Always set the tick generator to 12 cycles (1 us at clk_ref 12 MHz).
	   After power-on it is already enabled with CYCLES = 0 (reset value
	   0x200), i.e. one tick per 512 cycles: TIMER then runs at 23,437 us
	   per second, so "start only if not running" left it 43x slow.
	   Found and tested on the car by Jin, 13 Nov (PROJECT CONTEXT v2). */
	out_w(S3_WDT_TICK, S3_WDT_TICK_ENABLE | S3_XOSC_MHZ);
}

EXPORT UW drv_track_now_us(void)
{
	return in_w(S3_TIMERAWL);
}

/* ---- front IR pair (kernel ADC driver) ---- */
LOCAL void adc_pin_init(UW pin)
{
	/* Analog input: no digital function, input buffer and pulls off. */
	out_w(S3_GPIO_CTRL(pin), S3_FUNCSEL_NULL);
	out_w(S3_PAD(pin) + REG_ALIAS_CLR, S3_PAD_IE | S3_PAD_PUE | S3_PAD_PDE);
}

EXPORT ER drv_track_adc_open(void)
{
	adc_pin_init(S3_PIN_LEFT_ADC);
	adc_pin_init(S3_PIN_RIGHT_ADC);
	s3_adc_dd = tk_opn_dev((UB *)"adca", TD_READ);
	return (s3_adc_dd < E_OK) ? (ER)s3_adc_dd : E_OK;
}

LOCAL UH adc_read(W ch)
{
	UW	val = 0U;
	SZ	asz = 0;
	ER	er;

	er = tk_srea_dev(s3_adc_dd, ch, &val, 1, &asz);
	return (er < E_OK) ? 0U : (UH)(val & 0x0FFFU);
}

EXPORT UH drv_track_read_left(void)  { return adc_read(S3_ADC_CH_LEFT); }
EXPORT UH drv_track_read_right(void) { return adc_read(S3_ADC_CH_RIGHT); }

/* ---- barcode IR, GP7: two edge sources ----
 *
 * 1. GPIO edge interrupt (IO_IRQ_BANK0): microsecond timestamps.
 * 2. Sampler: TIMER alarm 3 interrupt every S3_SAMPLE_US, reads GP7 and
 *    timestamps every level change (resolution 0.5 ms; the narrowest real
 *    element is >= 5 ms, so the decoder's ratios are unaffected).
 *
 * Only ONE source feeds the edge queue at a time (s3_bar_src).  Both run
 * on core 0 at the same NVIC priority, so neither preempts the other and
 * the queue keeps a single producer.  The sampler always counts level
 * changes, so the barcode task can see whether the GPIO interrupt fires
 * and switch to the sampler if it does not (drv_track_barcode_use_sampler).
 *
 * Both sources push an edge only when GP7 now differs from the level after
 * the last pushed edge.  That keeps bar/space parity right even if several
 * edges collapse into one interrupt (even count: nothing pushed, like the
 * decoder's chatter rule; odd count: one edge pushed).
 */
#define S3_SIO_GPIO_IN		(0xD0000004U)
#define S3_TIMER_ALARM3		(0x4005401CU)
#define S3_TIMER_INTR		(0x40054034U)
#define S3_TIMER_INTE		(0x40054038U)
#define S3_ALARM3_BIT		(1U << 3)
#define S3_INTNO_TIMER3		(3U)		/* TIMER_IRQ_3 */
#define S3_SAMPLE_US		(500U)
#define S3_SAMPLE_STABLE	(2U)	/* samples a new level must hold */
/*
 * 19 Nov (Aurelius, PC model in testing/subsystem3/pc/barsim.c): the IR
 * module's comparator has no hysteresis, so while the spot crosses a bar
 * edge GP7 chatters for as long as the reflectance sits near the
 * threshold. At line-follow speed (~65 mm/s) that is tens of ms, not us.
 * The car logs show it: "irq22336 smp24" = 22,336 GP7 edges for 24 real
 * changes, and INVALID frames with 2-3 ms "elements". Two equal samples in
 * a row happen by chance inside that chatter, so extra edges reached the
 * decoder: 30-edge barcodes arrived as 40-160 edges and read INVALID, and
 * worse the SLOWER the car went.
 *
 * S3_SAMPLE_INTEG > 0 replaces the 2-sample rule with an up/down counter
 * (0..S3_SAMPLE_INTEG): +1 per sample that reads 1, -1 per sample that
 * reads 0. The level only changes when the counter reaches the other end,
 * so chatter has to become a clear majority first. 8 = 4 ms: the narrowest
 * bar (3 mm) is 46 ms at 65 mm/s and 7.5 ms at 400 mm/s, so nothing real
 * is lost. Model: 26 letters x 8 passes, noisy comparator, 65 mm/s:
 * 2-sample rule 9..195 of 208 read; counter 8: 208 of 208, 0 wrong.
 * Set to 0 for the old 2-sample rule.
 */
#ifndef S3_SAMPLE_INTEG
#define S3_SAMPLE_INTEG		(8U)
#endif

EXPORT volatile UW	s3_bar_src = S3_BAR_SRC_SAMPLER;
EXPORT volatile UW	s3_bar_raw_changes = 0U;	/* GP7 changed at all (before debounce) */
EXPORT volatile UW	s3_bar_isr_count = 0U;	/* GP7 interrupts taken */
EXPORT volatile UW	s3_bar_samp_changes = 0U;	/* level changes seen by the sampler */
EXPORT volatile UW	s3_bar_pushed = 0U;	/* edges put in the queue */
EXPORT ER		s3_bar_irq_er = E_OK;	/* GPIO interrupt set-up result */
EXPORT ER		s3_bar_samp_er = E_OK;	/* sampler set-up result */

/*
 * 20 Nov (Aurelius, home loop test): DISTANCE clock for the barcode.
 * The car log of 20 Nov 02:50 shows complete 30-edge barcodes that still
 * read INVALID with widths like 10 60 8 22 68 100 56 20 10 ms: the right
 * bars in the right order, stretched and squeezed. Line following weaves
 * (+-50..60 deg/s) and the barcode sensor sits out on the left arm, so its
 * speed over the bars swings between about 2 and 18 cm/s inside one
 * barcode. The decoder compares widths in TIME, which needs a steady
 * speed. PC model (testing/subsystem3/pc/barsim_weave.c): time clock reads
 * 12 of 156 at a +-40 % swing and 0 at +-60 %; distance clock 156 of 156
 * up to +-80 % when the encoders see at least 75 % of the swing.
 *
 * S3_BAR_DIST_CLOCK 1: edges are stamped with how far the SENSOR has
 * travelled instead of the time, from the wheel encoders, in units where
 * 10 = 1 um (so 1,000,000 = 100 mm, about what 1 s is at 10 cm/s, and the
 * decoder's limits keep their meaning: 2 ms chatter rule = 0.2 mm).
 * Sensor travel = left wheel + (left - right) x (arm - half track)/track:
 * the sensor is S3_BAR_ARM_MM left of the car's centre line, further out
 * than the left wheel. Travel is counted in both directions (absolute),
 * so the clock never runs backwards. While the car stands still the clock
 * stands still: a half-read barcode simply waits.
 */
#ifndef S3_BAR_DIST_CLOCK
#define S3_BAR_DIST_CLOCK	1	/* 1 = wheel distance (20 Nov lab default); 0 = time, as v19 */
#endif
#ifndef S3_BAR_ARM_MM
#define S3_BAR_ARM_MM		90	/* barcode sensor to the car's centre line (photo estimate; MEASURE) */
#endif
#define S3_BAR_HALF_TRACK_MM	55	/* AV_TRACK_MM 110 / 2 */
#define S3_BAR_ARM_K256		(((S3_BAR_ARM_MM - S3_BAR_HALF_TRACK_MM) * 256) / (2 * S3_BAR_HALF_TRACK_MM))
LOCAL volatile UW	s_bar_clock = 0U;	/* sensor travel, 10 units per um (wraps) */

LOCAL volatile UW	s_q_level;	/* GP7 level after the last pushed edge */
LOCAL volatile UW	s_samp_level;	/* GP7 level at the last sample */
LOCAL volatile UW	s_raw_level;	/* GP7 level of the previous raw sample */

LOCAL UW gp7_level(void)
{
	return (in_w(S3_SIO_GPIO_IN) >> S3_PIN_BARCODE) & 1U;
}

/* Interrupt context only (either source, same core, no nesting). */
LOCAL void push_if_changed(UW level, UW now)
{
	if (level != s_q_level) {
		s_q_level = level;
		if (barcode_eq_push(&s3_edge_q, now) != 0U) {
			s3_bar_pushed++;
		}
	}
}

LOCAL void gpio_bank0_inthdr(UINT intno)
{
	UW const core = in_w(S3_SIO_CPUID);
	UW const ev   = (in_w(S3_IO_PROC_INTS(core, S3_PIN_BARCODE))
			 >> S3_IO_SHIFT(S3_PIN_BARCODE)) & 0xFU;
	UW const now  = in_w(S3_TIMERAWL);

	(void)intno;
	/* Edge bits are write-1-to-clear: clear only ours. */
	out_w(S3_IO_INTR(S3_PIN_BARCODE),
	      (ev & S3_EVT_EDGES) << S3_IO_SHIFT(S3_PIN_BARCODE));
#if APP_WITH_S5
	{	/* GP5: Joe's ultrasonic echo (subsystem 5), same shared handler */
		UW const ev5 = (in_w(S3_IO_PROC_INTS(core, 5U)) >> S3_IO_SHIFT(5U)) & 0xFU;

		out_w(S3_IO_INTR(5U), (ev5 & S3_EVT_EDGES) << S3_IO_SHIFT(5U));
		if ((ev5 & S3_EVT_EDGES) != 0U) {
			drv_hcsr04_edge(ev5);
		}
	}
#endif
	/* GP0/GP1, GP16/GP17: Sean's wheel encoders (subsystem 2) */
	drv_encoder_irq(core);
	if ((ev & S3_EVT_EDGES) == 0U) {
		return;
	}
	s3_bar_isr_count++;
	/* Raw activity: count the edge bits the bank reported, before any
	   filtering, so "pin never moved" and "filter ate it" stay distinct. */
	{
		UW const e = ev & S3_EVT_EDGES;

		if ((e & S3_EVT_EDGE_LOW)  != 0U) { s3_bar_raw_changes++; }
		if ((e & S3_EVT_EDGE_HIGH) != 0U) { s3_bar_raw_changes++; }
	}
	if (s3_bar_src == S3_BAR_SRC_IRQ) {
		push_if_changed(gp7_level(), now);
	}
}

LOCAL void timer3_inthdr(UINT intno)
{
	static UW	cand_n  = 0U;	/* samples in a row at the new level */
	static UW	cand_us = 0U;	/* time of the first of them         */
	UW const	now = in_w(S3_TIMERAWL);
	UW		level;

	(void)intno;
	out_w(S3_TIMER_INTR, S3_ALARM3_BIT);		/* write 1 to clear */
	out_w(S3_TIMER_ALARM3, now + S3_SAMPLE_US);	/* re-arm */

#if S3_BAR_DIST_CLOCK
	{	/* advance the distance clock (same core and priority as the
		   encoder interrupt, so the counts cannot change under us) */
		static W	prev_l = 0, prev_r = 0;
		static BOOL	have_prev = FALSE;
		static UW	rem = 0U;
		static W	pend = 0;	/* signed travel not yet counted, Q8 counts */
		W		cl, cr, dl, dr, ds;

		drv_encoder_read(&cl, &cr);
		if (!have_prev) {
			prev_l = cl; prev_r = cr; have_prev = TRUE;
		}
		dl = cl - prev_l; dr = cr - prev_r;
		prev_l = cl; prev_r = cr;
#if ENC_RIGHT_ENABLED
		ds = (dl * 256) + ((dl - dr) * (W)S3_BAR_ARM_K256);
#else
		(void)dr;
		ds = dl * 256;
#endif
		/* Sum with the SIGN first and only count whole encoder steps.
		   The first version took |ds| on every 0.5 ms tick: the left
		   and right counts rarely land on the same tick, so the
		   "- right" part was counted as "+ right" and the clock ran
		   up to 4x wrong while weaving (clock_test.c, 20 Nov 03:05). */
		pend += ds;
		if ((pend >= 256) || (pend <= -256)) {
			rem += (UW)((pend < 0) ? -pend : pend) * 820U;	/* counts x 82 um x 10, Q8 */
			s_bar_clock += rem >> 8;
			rem &= 0xFFU;
			pend = 0;
		}
	}
#endif

	/* Debounce: a new level counts only after S3_SAMPLE_STABLE samples in
	   a row; its timestamp is the FIRST of them, so widths stay exact to
	   one sample.  Comparator bounce (us) and flicker < 0.5 ms vanish. */
	level = gp7_level();
	/* Raw activity, counted before the debounce filter below: if this
	   rises while s3_bar_samp_changes does not, the bars are shorter
	   than S3_SAMPLE_STABLE samples and the filter is eating them. */
	if (level != s_raw_level) {
		s_raw_level = level;
		s3_bar_raw_changes++;
	}
#if (S3_SAMPLE_INTEG > 0U)
	{
		static UW	acc = 0U;	/* 0..S3_SAMPLE_INTEG           */
		static BOOL	primed = FALSE;	/* acc started at the idle level */

		(void)cand_n;
		(void)cand_us;
		if (!primed) {
			acc    = (s_samp_level != 0U) ? S3_SAMPLE_INTEG : 0U;
			primed = TRUE;
		}
		if (level != 0U) {
			if (acc < S3_SAMPLE_INTEG) { acc++; }
		} else {
			if (acc > 0U) { acc--; }
		}
		if (((acc == S3_SAMPLE_INTEG) && (s_samp_level == 0U)) ||
		    ((acc == 0U) && (s_samp_level != 0U))) {
			s_samp_level = (s_samp_level != 0U) ? 0U : 1U;
			s3_bar_samp_changes++;
			if (s3_bar_src == S3_BAR_SRC_SAMPLER) {
				/* The counter lags a clean edge by S3_SAMPLE_INTEG
				   samples; half of that is the best estimate inside
				   chatter. Same offset on every edge, so widths are
				   unchanged. */
#if S3_BAR_DIST_CLOCK
				push_if_changed(s_samp_level, s_bar_clock);
#else
				push_if_changed(s_samp_level,
						now - ((S3_SAMPLE_INTEG * S3_SAMPLE_US) / 2U));
#endif
			}
		}
		return;
	}
#endif
	if (level == s_samp_level) {
		cand_n = 0U;
		return;
	}
	if (cand_n == 0U) {
		cand_us = now;
	}
	cand_n++;
	if (cand_n < S3_SAMPLE_STABLE) {
		return;
	}
	cand_n = 0U;
	s_samp_level = level;
	s3_bar_samp_changes++;
	if (s3_bar_src == S3_BAR_SRC_SAMPLER) {
		push_if_changed(level, cand_us);
	}
}

LOCAL ER gpio_irq_init(UW core)
{
	T_DINT	dint = {0};
	ER	er;

	/* Drop stale edges, then hook the bank interrupt on THIS core. */
	out_w(S3_IO_INTR(S3_PIN_BARCODE), S3_EVT_EDGES << S3_IO_SHIFT(S3_PIN_BARCODE));

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_bank0_inthdr;
	er = tk_def_int(S3_INTNO_IO_BANK0, &dint);
	if (er < E_OK) {
		return er;
	}
	ClearInt(S3_INTNO_IO_BANK0);
	EnableInt(S3_INTNO_IO_BANK0, S3_GPIO_INTPRI);

	/* Enable both edges of GP7 for this core (SET alias: the register
	   also holds GP0-GP6 bits that other subsystems may own). */
	out_w(S3_IO_PROC_INTE(core, S3_PIN_BARCODE) + REG_ALIAS_SET,
	      S3_EVT_EDGES << S3_IO_SHIFT(S3_PIN_BARCODE));
	return E_OK;
}

LOCAL ER sampler_init(void)
{
	T_DINT	dint = {0};
	ER	er;

	out_w(S3_TIMER_INTE + REG_ALIAS_CLR, S3_ALARM3_BIT);
	out_w(S3_TIMER_INTR, S3_ALARM3_BIT);

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)timer3_inthdr;
	er = tk_def_int(S3_INTNO_TIMER3, &dint);
	if (er < E_OK) {
		return er;
	}
	ClearInt(S3_INTNO_TIMER3);
	EnableInt(S3_INTNO_TIMER3, S3_GPIO_INTPRI);	/* same level: no nesting */

	out_w(S3_TIMER_INTE + REG_ALIAS_SET, S3_ALARM3_BIT);
	out_w(S3_TIMER_ALARM3, in_w(S3_TIMERAWL) + S3_SAMPLE_US);	/* arms it */
	return E_OK;
}

EXPORT ER drv_track_barcode_irq_init(void)
{
	UW const	core = in_w(S3_SIO_CPUID);

	/* Input with pull-up, so an unplugged sensor reads a steady 1. */
	gpio_set_pin(S3_PIN_BARCODE, GPIO_MODE_IN);
	out_w(S3_PAD(S3_PIN_BARCODE) + REG_ALIAS_CLR, S3_PAD_PDE);
	out_w(S3_PAD(S3_PIN_BARCODE) + REG_ALIAS_SET, S3_PAD_PUE);

	s_q_level    = gp7_level();
	s_samp_level = s_q_level;
	s_raw_level  = s_q_level;

	s3_bar_samp_er = sampler_init();
	s3_bar_irq_er  = gpio_irq_init(core);

	if ((s3_bar_irq_er < E_OK) && (s3_bar_samp_er >= E_OK)) {
		drv_track_barcode_use_sampler();
	}
	return (s3_bar_irq_er < E_OK) ? s3_bar_irq_er : s3_bar_samp_er;
}

/* Barcode task (core 0): stop using the GPIO interrupt, feed the queue
   from the sampler from now on.  The level is re-synced with interrupts
   off so the first sampled change is a real one. */
EXPORT void drv_track_barcode_use_sampler(void)
{
	UW const	core = in_w(S3_SIO_CPUID);
	UINT		imask;

	out_w(S3_IO_PROC_INTE(core, S3_PIN_BARCODE) + REG_ALIAS_CLR,
	      S3_EVT_EDGES << S3_IO_SHIFT(S3_PIN_BARCODE));
	DI(imask);
	s_q_level   = gp7_level();
	s_raw_level = s_q_level;
	s3_bar_src  = S3_BAR_SRC_SAMPLER;
	EI(imask);
}

/* The clock the barcode edges are stamped with: time (us), or sensor
   travel when S3_BAR_DIST_CLOCK is 1. The barcode task passes this, not
   the time, to the decoder's "time passed" check. */
EXPORT UW drv_track_bar_clock(void)
{
#if S3_BAR_DIST_CLOCK
	return s_bar_clock;
#else
	return in_w(S3_TIMERAWL);
#endif
}

EXPORT UW drv_track_barcode_level(void)
{
	return gpio_get_val(S3_PIN_BARCODE);
}

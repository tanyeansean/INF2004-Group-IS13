/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoder driver (quadrature, x4).  See drv_encoder.h.
 *
 *	ISR <-> tasks: only drv_encoder_irq() (core 0, interrupt context)
 *	writes the counters.  Each one is an aligned 32-bit word, so a task on
 *	either core reads it in one load without locking.
 */

#include <sys/machine.h>
#include "drv_encoder.h"

#define REG_ALIAS_SET		(0x2000U)
#define REG_ALIAS_CLR		(0x3000U)

#define ENC_SIO_GPIO_IN		(0xD0000004U)
#define ENC_IO_BANK0		(0x40014000U)
#define ENC_IO_INTR(pin)	(ENC_IO_BANK0 + 0x0F0U + (4U * ((pin) / 8U)))
#define ENC_IO_PROC_INTE(c, pin) (ENC_IO_BANK0 + 0x100U + (0x30U * (c)) + (4U * ((pin) / 8U)))
#define ENC_IO_PROC_INTS(c, pin) (ENC_IO_BANK0 + 0x120U + (0x30U * (c)) + (4U * ((pin) / 8U)))
#define ENC_IO_SHIFT(pin)	(4U * ((pin) % 8U))
#define ENC_EVT_EDGES		((1U << 2) | (1U << 3))	/* EDGE_LOW | EDGE_HIGH */

#define ENC_PADS_BANK0		(0x4001C000U)
#define ENC_PAD(pin)		(ENC_PADS_BANK0 + 0x04U + (4U * (pin)))
#define ENC_PAD_PDE		(1U << 2)
#define ENC_PAD_PUE		(1U << 3)

#define ENC_IRQ_CORE		(0U)	/* IO_IRQ_BANK0 is enabled on core 0 */
#define ENC_BAD			(2)	/* table marker: both phases changed */

/* Step for (previous AB << 2) | current AB, AB = (A << 1) | B.
   00 -> 01 -> 11 -> 10 -> 00 is +1 per step, the reverse is -1, and a
   jump of both phases at once (a missed edge) is ENC_BAD. */
LOCAL const B	enc_step[16] = {
	 0, +1, -1, ENC_BAD,
	-1,  0, ENC_BAD, +1,
	+1, ENC_BAD,  0, -1,
	ENC_BAD, -1, +1,  0
};

typedef struct {
	UW		pin_a;
	UW		pin_b;
	W		sign;		/* +1, or -1 for ENC_INVERT_* */
	volatile W	count;		/* written by the ISR only */
	volatile UW	edges;		/* interrupts that saw a change */
	volatile UW	bad;		/* invalid transitions (missed edges) */
	UW		prev;		/* last AB state, ISR only after init */
} enc_wheel_t;

LOCAL enc_wheel_t	enc_left = {
	ENC_PIN_LEFT_A, ENC_PIN_LEFT_B, (ENC_INVERT_LEFT != 0) ? -1 : 1, 0, 0U, 0U, 0U
};
LOCAL enc_wheel_t	enc_right = {
	ENC_PIN_RIGHT_A, ENC_PIN_RIGHT_B, (ENC_INVERT_RIGHT != 0) ? -1 : 1, 0, 0U, 0U, 0U
};

LOCAL UW ab_state(const enc_wheel_t *w, UW gpio_in)
{
	return (((gpio_in >> w->pin_a) & 1U) << 1) | ((gpio_in >> w->pin_b) & 1U);
}

/* Read and clear (write 1) the edge bits of one pin for this core. */
LOCAL UW take_edges(UW core, UW pin)
{
	UW const ev = (in_w(ENC_IO_PROC_INTS(core, pin)) >> ENC_IO_SHIFT(pin)) & ENC_EVT_EDGES;

	if (ev != 0U) {
		out_w(ENC_IO_INTR(pin), ev << ENC_IO_SHIFT(pin));
	}
	return ev;
}

LOCAL void pin_setup(UW pin)
{
	/* Input with pull-up: harmless with push-pull outputs, needed with
	   open-collector ones, and an unplugged encoder reads a steady 1. */
	gpio_set_pin(pin, GPIO_MODE_IN);
	out_w(ENC_PAD(pin) + REG_ALIAS_CLR, ENC_PAD_PDE);
	out_w(ENC_PAD(pin) + REG_ALIAS_SET, ENC_PAD_PUE);
}

LOCAL void wheel_init(enc_wheel_t *w)
{
	pin_setup(w->pin_a);
	pin_setup(w->pin_b);
	w->prev = ab_state(w, in_w(ENC_SIO_GPIO_IN));

	/* Drop stale edges, then enable both edges of A and B on core 0
	   (SET alias: the INTE words also hold other subsystems' pins). */
	out_w(ENC_IO_INTR(w->pin_a), ENC_EVT_EDGES << ENC_IO_SHIFT(w->pin_a));
	out_w(ENC_IO_INTR(w->pin_b), ENC_EVT_EDGES << ENC_IO_SHIFT(w->pin_b));
	out_w(ENC_IO_PROC_INTE(ENC_IRQ_CORE, w->pin_a) + REG_ALIAS_SET,
	      ENC_EVT_EDGES << ENC_IO_SHIFT(w->pin_a));
	out_w(ENC_IO_PROC_INTE(ENC_IRQ_CORE, w->pin_b) + REG_ALIAS_SET,
	      ENC_EVT_EDGES << ENC_IO_SHIFT(w->pin_b));
}

/* Interrupt context: after the edge bits are cleared, the pins are read
   once, so an edge arriving in between is still seen in the new state. */
LOCAL void wheel_update(enc_wheel_t *w, UW core)
{
	UW	ev;
	UW	now;
	B	step;

	ev  = take_edges(core, w->pin_a);
	ev |= take_edges(core, w->pin_b);
	if (ev == 0U) {
		return;
	}
	now  = ab_state(w, in_w(ENC_SIO_GPIO_IN));
	step = enc_step[(w->prev << 2) | now];
	w->prev = now;
	if (step == ENC_BAD) {
		w->bad++;
	} else if (step != 0) {
		w->count += w->sign * (W)step;
		w->edges++;
	}
}

EXPORT ER drv_encoder_init(void)
{
	wheel_init(&enc_left);
#if ENC_RIGHT_ENABLED
	wheel_init(&enc_right);
#endif
	return E_OK;
}

EXPORT void drv_encoder_irq(UW core)
{
	wheel_update(&enc_left, core);
#if ENC_RIGHT_ENABLED
	wheel_update(&enc_right, core);
#endif
}

EXPORT void drv_encoder_read(W *p_left, W *p_right)
{
	*p_left  = enc_left.count;
	*p_right = enc_right.count;
}

/* Raw pin levels now, (A << 1) | B per wheel: lets a test tell "the pins
   never change" (wiring/power) from "they change but are not counted". */
EXPORT void drv_encoder_levels(UW *p_ab_l, UW *p_ab_r)
{
	UW const gpio_in = in_w(ENC_SIO_GPIO_IN);

	*p_ab_l = ab_state(&enc_left, gpio_in);
	*p_ab_r = ab_state(&enc_right, gpio_in);
}

EXPORT void drv_encoder_diag(UW *p_edges_l, UW *p_edges_r, UW *p_bad_l, UW *p_bad_r)
{
	*p_edges_l = enc_left.edges;
	*p_edges_r = enc_right.edges;
	*p_bad_l   = enc_left.bad;
	*p_bad_r   = enc_right.bad;
}

/*
 *	task_motor.c
 *	Subsystem 2 -- Motor Control task: the ONLY task that writes the motor
 *	PWM.  Applies each wheel command from Mission Control as soon as it
 *	arrives (same latency as integrated_car, where the main loop called
 *	motor_set() directly), and wakes at least every 20 ms for its heartbeat.
 *
 *	Odometry: every >= 20 ms it turns the wheel encoder counts into
 *	distance and speed per wheel and publishes them (s2_odom_get()).
 *	Still open loop: Sean's 20 ms per-wheel PID uses these speeds later.
 */

#include "subsystem2.h"
#include "../subsystem1/subsystem1.h"	/* flg_hb */
#include "../subsystem3/drv_track.h"	/* drv_track_now_us(): 1 us TIMER */

#define S2_MOTOR_WAIT_MS	20

#define S2_DMB()	__asm__ volatile ("dmb" ::: "memory")

/* Published odometry: written only by motor_task, read by any task.
   Sequence lock: odd = being written.  The writer disables dispatching
   on its core while it writes, so a reader on the same core never sees
   an odd sequence; a reader on the other core retries for a few us. */
LOCAL s2_odom_t		s_odom;
LOCAL volatile UW	s_odom_seq = 0U;

LOCAL void odom_publish(const s2_odom_t *o)
{
	(void)tk_dis_dsp();
	s_odom_seq = s_odom_seq + 1U;
	S2_DMB();
	s_odom = *o;
	S2_DMB();
	s_odom_seq = s_odom_seq + 1U;
	(void)tk_ena_dsp();
}

EXPORT void s2_odom_get(s2_odom_t *p)
{
	UW	s1, s2;

	do {
		s1 = s_odom_seq;
		S2_DMB();
		*p = s_odom;
		S2_DMB();
		s2 = s_odom_seq;
	} while ((s1 != s2) || ((s1 & 1U) != 0U));
}

/* Speed over a window: um travelled / us elapsed, in mm/s.  32 bits are
   enough: dt <= S2_ODOM_MAX_WINDOW_US (0.2 s), so |d_um| * 1000 overflows
   only above 10 m/s. */
LOCAL W speed_mms(W d_um, UW dt_us)
{
	return (d_um * 1000) / (W)dt_us;
}

/* Called on every wake-up; samples once about S2_ODOM_PERIOD_US has
   passed since the last sample.  The 20 ms receive timeout is counted in
   10 ms kernel ticks, so "20 ms" can read a little under 20000 us on the
   1 us timer: the jitter margin stops that wake-up from being skipped
   (which would double the window). */
LOCAL void odom_update(s2_odom_t *o)
{
	UW const	now = drv_track_now_us();
	UW const	dt  = now - o->t_us;	/* wraps correctly (unsigned) */
	W		cl, cr;

	if ((o->updates != 0U) && (dt < (S2_ODOM_PERIOD_US - S2_ODOM_JITTER_US))) {
		return;
	}
	drv_encoder_read(&cl, &cr);

	if ((o->updates != 0U) && (dt <= S2_ODOM_MAX_WINDOW_US)) {
		o->speed_mms_l = speed_mms(s2_counts_to_um(cl - o->count_l), dt);
		o->speed_mms_r = speed_mms(s2_counts_to_um(cr - o->count_r), dt);
		o->window_us   = dt;
	}
	/* First sample, or the task was starved: restart the window and keep
	   the previous speeds rather than average over a long gap. */
	o->count_l   = cl;
	o->count_r   = cr;
	o->dist_mm_l = s2_counts_to_um(cl) / 1000;
	o->dist_mm_r = s2_counts_to_um(cr) / 1000;
	o->t_us      = now;
	o->updates++;
	odom_publish(o);
}

EXPORT void motor_task(INT stacd, void *exinf)
{
	s2_odom_t	odom = {0};
	UW		m;
	INT		n;

	(void)stacd; (void)exinf;

	for (;;) {
		n = tk_rcv_mbf(mbf_motor, &m, S2_MOTOR_WAIT_MS);
		if (n == (INT)sizeof(m)) {
			drv_motor_set(s2_wheel_get(m, 0U), s2_wheel_get(m, S2_WHEEL_BITS));
		}
		odom_update(&odom);
		(void)tk_set_flg(flg_hb, HB_MOTOR_CONTROL);
	}
}

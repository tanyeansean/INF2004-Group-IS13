/*
 *	subsystem2.h
 *	Subsystem 2 -- Mission Control + Motor Control (Buddy 2, Tan Yean Sean).
 *	Student ID: 2500224.
 *
 *	uT-Kernel port of the car logic that ran in integrated_car/main.c and
 *	motor.c: straddle steering, junction stop / barcode turns, start / stop
 *	/ speed, open-loop PWM.  No encoders or PID yet: Motor Control drives
 *	the PWM it is given (motor.c stand-in) until Sean's 20 ms PID replaces
 *	it behind the same queue.
 *
 *	Inputs, owned here (the receiver creates the object):
 *	  IF-01  mbf_line_err  line_err_t, 2 bytes, every 5 ms   (from S3)
 *	  IF-02  mbf_barcode   barcode_t, 1 byte, per read       (from S3)
 *	  IF-04  mbf_cmd       comms_cmd_t, 1 byte, per command  (from S1)
 *	Shared data: g_nav_map (nav_cmd.h): written only by Telemetry with
 *	nav_map_apply() (one 32-bit store), read here with nav_map_snapshot().
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "shared_types.h"	/* contract: line_err_q, barcode_q, cmd_q, faults */

#define S2_PRI_MOTOR		14	/* band 14-15 (TEAM.md) */
#define S2_PRI_MISSION		15

IMPORT ID	mbf_line_err;	/* IF-01 */
IMPORT ID	mbf_barcode;	/* IF-02 */
IMPORT ID	mbf_cmd;	/* IF-04 */
IMPORT nav_map_t	g_nav_map;

/* IF-06 (S5 -> S2): obstacle_q.  OBSTACLE_MSG_SIZE (6) bytes per message,
   packed by obstacle_msg_pack() (subsystem5/obstacle_msg.h).  Owned here,
   the receiver.  Mission Control drains it every pass and answers on S5's
   flg_avoid (AVOID_EVT_MOTORS_STOPPED / LINE_REACQUIRED / RESUME). */
IMPORT ID	mbf_obstacle;

/* Shared 16-bit fault flag (shared_types.h section 1, G1/G2/C5).  Each task
   sets ONLY its own FAULT_BIT_* with tk_set_flg(); Mission Control reads
   it every pass and is the only task that clears bits. */
IMPORT ID	flg_faults;

/* ---------------------------------------------------------------------
 *  Private to subsystem 2: Mission Control -> Motor Control (mbf_motor).
 *  Open-loop stand-in for Sean's motor_cmd_q until encoders + PID exist.
 *  One uint32_t (Change Requirement 1): bits 0-8 left PWM, bits 9-17
 *  right PWM, each signed 9-bit (-255..+255, + = forward).
 * --------------------------------------------------------------------- */
#define S2_WHEEL_BITS		9U
#define S2_WHEEL_MASK		0x1FFU
#define S2_PWM_MAX		255

static inline UW s2_wheels_pack(H left, H right)
{
	if (left > S2_PWM_MAX)   { left = S2_PWM_MAX; }
	if (left < -S2_PWM_MAX)  { left = -S2_PWM_MAX; }
	if (right > S2_PWM_MAX)  { right = S2_PWM_MAX; }
	if (right < -S2_PWM_MAX) { right = -S2_PWM_MAX; }
	return ((UW)left & S2_WHEEL_MASK) | (((UW)right & S2_WHEEL_MASK) << S2_WHEEL_BITS);
}

static inline H s2_wheel_get(UW m, UW shift)
{
	UW v = (m >> shift) & S2_WHEEL_MASK;
	return (H)(((v & 0x100U) != 0U) ? ((INT)v - 512) : (INT)v);
}

IMPORT ID	mbf_motor;
IMPORT void	motor_task(INT stacd, void *exinf);
IMPORT void	mission_task(INT stacd, void *exinf);

/* drv_motor.c, called only by motor_task (and once at init, motors off) */
IMPORT void	drv_motor_init(void);
IMPORT void	drv_motor_set(H left, H right);

#endif /* SUBSYSTEM2_H */

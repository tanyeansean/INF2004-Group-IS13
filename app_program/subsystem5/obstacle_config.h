/*
 *	obstacle_config.h
 *	Subsystem 5 -- pins, timing and tuning constants (private).
 *
 *	Every value marked TUNE is a starting guess: replace it with a
 *	measured value (assumption A3 and the Week 10 metrics).
 *	No kernel or board headers here, so the pure modules stay host-testable.
 */

#ifndef OBSTACLE_CONFIG_H
#define OBSTACLE_CONFIG_H

/* ---------------------------------------------------------------- Pins */
/* Claim these in TEAM.md section 2 before wiring.                       */
#define OA_PIN_TRIG		(4U)	/* HC-SR04P TRIG                          */
#define OA_PIN_ECHO		(5U)	/* HC-SR04P ECHO. Plain HC-SR04 is 5 V:
					   needs 5 V supply + divider on ECHO.     */
#define OA_PIN_SERVO		(12U)	/* PWM slice 6 channel A. GP13 shares
					   slice 6: it must also run at 50 Hz.     */

/* ---------------------------------------------------------------- HC-SR04 */
#define OA_PING_PERIOD_MS	(60U)	/* Datasheet: >= 60 ms between triggers.  */
#define OA_ECHO_TIMEOUT_MS	(30U)	/* 254 cm = 14.7 ms echo. Kernel tick is
					   10 ms (CNF_TIMER_PERIOD), so this is
					   really 30-40 ms.                         */
#define OA_US_PER_CM		(58U)	/* Round trip at ~343 m/s.                */
#define OA_DIST_MIN_CM		(2U)
#define OA_DIST_MAX_CM		(254U)
#define OA_DIST_OUT_OF_RANGE	(255U)	/* Only meaning: beyond 254 cm.           */
#define OA_NO_ECHO_LIMIT	(5U)	/* Consecutive no-echo pings -> fault.    */

/* ---------------------------------------------------------------- Detection */
#define OA_STOP_DIST_CM		(25U)	/* TUNE: obstacle confirmed below this.   */
#define OA_CONFIRM_COUNT	(2U)	/* Consecutive readings to confirm.       */

/* ---------------------------------------------------------------- Servo */
#define OA_SERVO_CENTRE_DEG	(90U)
#define OA_SERVO_MIN_DEG	(15U)
#define OA_SERVO_MAX_DEG	(165U)
#define OA_SERVO_PULSE_MIN_US	(500U)	/* TUNE: pulse at 0 deg.                  */
#define OA_SERVO_PULSE_MAX_US	(2500U)	/* TUNE: pulse at 180 deg.                */
#define OA_SERVO_MS_PER_DEG	(2U)	/* SG90 ~0.1 s / 60 deg = 1.7 ms/deg.     */
#define OA_SERVO_SETTLE_MS	(30U)	/* TUNE: extra wait after a move.         */
#define OA_SERVO_LEFT_IS_HIGH	(1)	/* 1: angle > 90 points LEFT. VERIFY.     */

/* ---------------------------------------------------------------- Scanning */
#define OA_COARSE_START_DEG	(30U)	/* 30, 60, 90, 120, 150 deg.              */
#define OA_COARSE_STEP_DEG	(30U)
#define OA_COARSE_COUNT		(5U)
#define OA_FINE_STEP_DEG	(5U)	/* TUNE: smaller = slower, sharper edges. */
#define OA_FINE_MARGIN_DEG	(20U)	/* TUNE: scan this far past coarse hits.  */
#define OA_OBSTACLE_RANGE_CM	(60U)	/* TUNE: a reading this close is a "hit". */
#define OA_CLUSTER_JUMP_CM	(15U)	/* TUNE: bigger jump = different object.  */
#define OA_TAN_HALF_BEAM_Q15	(4314)	/* TUNE: tan(7.5 deg) * 32768.            */

/* ---------------------------------------------------------------- Planning */
#define OA_CAR_WIDTH_CM		(15U)	/* MEASURE the real car.                  */
#define OA_SIDE_MARGIN_CM	(5U)	/* TUNE                                   */
#define OA_MAX_RETRIES		(2U)	/* Reverse-and-rescan attempts (2 bits).  */

/* ---------------------------------------------------------------- Waits */
#define OA_WAIT_CHUNK_MS	(50U)	/* <= 60 ms, agreed for the heartbeat.    */
#define OA_STOP_ACK_TIMEOUT_MS	(500U)
#define OA_REVERSE_TIMEOUT_MS	(3000U)
#define OA_BYPASS_TIMEOUT_MS	(5000U)	/* Week 10 grade A: < 3 s in total.       */
#define OA_BYPASS_POLL_MS	(20U)
#define OA_BYPASS_ABORT_CM	(10U)	/* TUNE: too close during bypass.         */

/*
 * Optional timing probe (Lab 3/5: a GPIO write costs ~30 ns, tm_printf
 * costs ms and changes the timing). -1 = off, or build with
 * make ... EXTRA_CFLAGS=-DOA_PROBE_PIN=15 after adding that hook, or edit.
 * HIGH from "obstacle confirmed" until the plan is sent (or CLEAR/FAULT).
 */
#ifndef OA_PROBE_PIN
#define OA_PROBE_PIN		(-1)
#endif

#endif /* OBSTACLE_CONFIG_H */

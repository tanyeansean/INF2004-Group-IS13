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
/* TUNE: echoes shorter than this are rejected as noise (false near-field
   echo / crosstalk, seen on the car as phantom 3-4 cm "obstacles"). The
   car stops at OA_STOP_DIST_CM, so nothing real is ever this close.
   Must stay below OA_BYPASS_ABORT_CM. */
#define OA_DIST_MIN_VALID_CM	(6U)

/* ---------------------------------------------------------------- Detection */
/* TUNE: obstacle confirmed below this. Must be far enough for the narrow
   sweep (+-25 deg) and the wide beam to see past a ~15 cm object: from
   25 cm every angle hit it (L=0 R=0); edges first showed at ~30 cm. */
#define OA_STOP_DIST_CM		(35U)
#define OA_CONFIRM_COUNT	(2U)	/* Consecutive readings to confirm.       */

/* ---------------------------------------------------------------- Servo */
#define OA_SERVO_CENTRE_DEG	(90U)
/* Sweep limited to centre +/- 30 deg: no physical room for more on the
   car. Every commanded angle is clamped to this range. Widen these (and
   the coarse scan below) once the mount allows it. */
#define OA_SERVO_MIN_DEG	(65U)	/* centre - 25: mount barely clears */
#define OA_SERVO_MAX_DEG	(115U)	/* centre + 25                      */
#define OA_SERVO_PULSE_MIN_US	(500U)	/* TUNE: pulse at 0 deg.                  */
#define OA_SERVO_PULSE_MAX_US	(2500U)	/* TUNE: pulse at 180 deg.                */
#define OA_SERVO_MS_PER_DEG	(2U)	/* SG90 ~0.1 s / 60 deg = 1.7 ms/deg.     */
#define OA_SERVO_SETTLE_MS	(30U)	/* TUNE: extra wait after a move.         */
#define OA_SERVO_LEFT_IS_HIGH	(1)	/* 1: angle > 90 points LEFT. VERIFY.     */

/*
 * 0 = servo DISABLED: no PWM pulses are ever sent, so the servo stays
 *     where it is. Every scan point then measures straight ahead, so
 *     width/clearance/LEFT-RIGHT are not meaningful.
 * 1 = servo enabled, sweeping only OA_SERVO_MIN_DEG..OA_SERVO_MAX_DEG.
 */
#ifndef OA_SERVO_ENABLE
#define OA_SERVO_ENABLE		(1)
#endif

/* ---------------------------------------------------------------- Scanning */
/* 65, 75, 85, 95, 105, 115 deg. 10 deg steps are finer than the ~15 deg
   beam, so the coarse scan leaves no blind gaps inside the narrow sweep. */
#define OA_COARSE_START_DEG	(65U)
#define OA_COARSE_STEP_DEG	(10U)
#define OA_COARSE_COUNT		(6U)
#define OA_FINE_STEP_DEG	(5U)	/* TUNE: smaller = slower, sharper edges. */
#define OA_FINE_MARGIN_DEG	(15U)	/* TUNE: scan this far past coarse hits.  */

#if (OA_COARSE_START_DEG < OA_SERVO_MIN_DEG) || \
    ((OA_COARSE_START_DEG + ((OA_COARSE_COUNT - 1U) * OA_COARSE_STEP_DEG)) > OA_SERVO_MAX_DEG)
#error "Coarse scan angles fall outside OA_SERVO_MIN_DEG..OA_SERVO_MAX_DEG."
#endif
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
#define OA_MOVE_TIMEOUT_MS	(8000U)	/* One motion_q move not finished in this
					   long: Mission Control is not answering. */

/* ------------------------------------------------- Bypass + line recovery */
/* S5 plans every leg; Mission Control drives each one (motion_q) and its
   open-loop distance / angle calibration decides how accurate they are. */
#define OA_REVERSE_CM		(15U)	/* TUNE: back off this far, then rescan.  */
#define OA_TURN_OUT_DEG		(45U)	/* Leave / rejoin the line at 45 deg.      */
#define OA_DEFAULT_SIDESTEP_CM	(20U)	/* If the profile gives none (255).        */
#define OA_PASS_EXTRA_CM	(25U)	/* TUNE: obstacle depth + car length: drive
					   this far past the obstacle's front.   */
#define OA_SEEK_FACTOR		(2U)	/* Look for the line up to 2x the expected
					   distance before the second search.    */
#define OA_SEEK_MAX_CM		(40U)	/* TUNE: second search, square to the line. */
#define OA_MOVE_MAX_CM		(127U)	/* motion_q param is 8-bit signed.         */
#define OA_BYPASS_ABORT_CM	(10U)	/* TUNE: too close during a forward leg.   */

/*
 * Optional timing probe (Lab 3/5: a GPIO write costs ~30 ns, tm_printf
 * costs ms and changes the timing). -1 = off, or build with
 * make ... EXTRA_CFLAGS=-DOA_PROBE_PIN=15 after adding that hook, or edit.
 * HIGH from "obstacle confirmed" until the plan is sent (or CLEAR/FAULT).
 */
/* 1: while the state machine runs (OA_SENSOR_PROBE 0), print at most one
   "[S5 dbg] state=.. ping=.." line per second from oa_ping(), plus the
   reason once whenever S5 faults. Nothing prints before Mission Control
   sends MC_READY. Set 0 once the sensor is trusted. */
#ifndef OA_DEBUG
#define OA_DEBUG		(1)
#endif

#ifndef OA_PROBE_PIN
#define OA_PROBE_PIN		(-1)
#endif

#endif /* OBSTACLE_CONFIG_H */

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
/* TUNE: obstacle confirmed below this. Lowered to 20 so the car stops
   closer. Trade-off: from 25 cm or less a ~15 cm object fills the whole
   +-25 deg sweep (both edges unseen, L=0 R=0, log 2026-11-02), so the
   first plan is usually REVERSE; the OA_REVERSE_CM back-off (now ended
   on the wheel encoders) brings the car to ~35 cm, where edges showed.
   Keep above OA_DIST_MIN_VALID_CM (checked in drv_hcsr04.c). */
#define OA_STOP_DIST_CM		(20U)
#define OA_CONFIRM_COUNT	(2U)	/* Consecutive readings to confirm.       */
/* Width test (added 19 Nov by Jin, task_obstacle.c oa_width_test): 1 = the
   servo sweeps every OA_WIDTH_TEST_PERIOD_MS whatever is in front and
   prints the scan + width; no stopping or avoiding.  TEST BUILDS ONLY. */
#ifndef OA_WIDTH_TEST
#define OA_WIDTH_TEST		(0)
#endif
#define OA_WIDTH_TEST_PERIOD_MS	(4000U)

/* ---------------------------------------------------------------- Servo */
/* The servo angle at which the sensor points STRAIGHT AHEAD. 90 -> 100
   (Jin, 19 Nov): the sensor sits a little tilted to the right on its
   mount (Jin), and the width test agreed: a centred 10 cm box at 20 cm
   showed up from ~27 deg right to ~47 deg left of 90, i.e. about 10 deg
   lopsided. 100 points it straight (angle > 90 = left). Every sideways
   calculation (obstacle_profile.c) measures angles from here, and the
   straight-ahead pings while driving use it. TUNE: a centred box should
   give a scan that is even on both sides. */
#define OA_SERVO_CENTRE_DEG	(100U)
/* Sweep: every commanded servo angle is clamped to this range.
   +/- 30 -> +/- 60 -> +/- 55 (Jin, 19 Nov) around the servo's own middle
   (90). The sensor is on a ~2 cm arm now; at +/- 30 a 15 cm box filled
   the whole sweep from ~40 cm (width test 19 Nov), so its edges were
   never seen and the car kept reversing. With straight ahead at 100,
   this sees 65 deg to the right and 45 deg to the left. */
#define OA_SERVO_MIN_DEG	(35U)	/* servo middle - 55 */
#define OA_SERVO_MAX_DEG	(145U)	/* servo middle + 55 */
/* How far the sensor face sits out from the servo's turning point (Jin,
   19 Nov: moved out by about 2 cm; before, the code took it as 0). Added
   to every reading in oa_ping(), so all distances are from the turning
   point. MEASURE: if the face was already ahead of the turning point
   before the change, set the whole distance here. */
#define OA_SENSOR_ARM_CM	(2U)
#define OA_SERVO_PULSE_MIN_US	(500U)	/* TUNE: pulse at 0 deg.                  */
#define OA_SERVO_PULSE_MAX_US	(2500U)	/* TUNE: pulse at 180 deg.                */
#define OA_SERVO_MS_PER_DEG	(2U)	/* SG90 ~0.1 s / 60 deg = 1.7 ms/deg.     */
#define OA_SERVO_SETTLE_MS	(50U)	/* TUNE: still time after a move, before
					   pinging (was 30: let the mount stop
					   shaking first).                        */
/* The servo is moved in small steps instead of one jump, so it turns
   gently: the SG90 alone swings at ~600 deg/s, which shook the sensor
   loose. OA_SERVO_SLEW_DEG every OA_SERVO_SLEW_MS (one 10 ms kernel tick,
   really 10-20 ms) = about 100-200 deg/s. TUNE: lower SLEW_DEG = slower. */
#define OA_SERVO_SLEW_DEG	(2U)
#define OA_SERVO_SLEW_MS	(10U)
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
/* 35, 45, ... 145 deg (was 60..120, Jin 19 Nov: wider sweep). 10 deg
   steps are finer than the ~15 deg beam, so the coarse scan leaves no
   blind gaps. */
#define OA_COARSE_START_DEG	(35U)
#define OA_COARSE_STEP_DEG	(10U)
#define OA_COARSE_COUNT		(12U)
#define OA_FINE_STEP_DEG	(5U)	/* TUNE: 1 deg was tried: ~5 s per scan,
					   no better edges (the wide beam blurs
					   them), and more lone no-echo misses
					   splitting the obstacle. Back to 5.     */
#define OA_FINE_MARGIN_DEG	(15U)	/* TUNE: scan this far past coarse hits.  */

/* Half the ultrasonic beam, MEASURED on the car (Jin, 19 Nov, 10 cm box,
   v14 width test: the box was still seen this far past its corners).
   Used by the width and edge maths in obstacle_profile.c in place of
   OA_TAN_HALF_BEAM_Q15 (7.5 deg). ~20 deg at 18 and 34 cm; ~31 deg at
   48 cm, so it grows by OA_BEAM_GROW_PER_CM_X100 / 100 deg per cm beyond
   OA_BEAM_GROW_FROM_CM. TUNE: re-measure if the sensor or mount changes. */
/* 20 -> 14 (Jin, 19 Nov evening): how far the beam "sees past" an edge
   depends on the obstacle. The flat 10 cm box gave ~20 deg, a makeshift
   10 cm obstacle only ~10 deg, so 20 read it as 3-5 cm. 14 is in between
   and errs WIDE: replaying the recorded scans, the makeshift one reads
   7-9 cm and the flat box 15-22 cm (true 10 for both). Reading an
   obstacle too narrow is the dangerous side (the car passes closer). */
#ifndef OA_BEAM_HALF_DEG
#define OA_BEAM_HALF_DEG	(14U)
#endif
#define OA_BEAM_GROW_FROM_CM	(35U)
#define OA_BEAM_GROW_PER_CM_X100 (85U)
#define OA_BEAM_HALF_MAX_DEG	(45U)

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
/* TUNE: extra sideways distance added to every sidestep, on top of
   half the car + OA_SIDE_MARGIN_CM. The ultrasonic width is only an
   estimate (wide beam), and the car grazed obstacles with 0 here. Only
   changes how far the car steps out, not when a side counts as open. */
#define OA_SIDESTEP_EXTRA_CM	(10U)
#define OA_MAX_RETRIES		(2U)	/* Reverse-and-rescan attempts (2 bits).  */

/* ---------------------------------------------------------------- Waits */
#define OA_WAIT_CHUNK_MS	(50U)	/* <= 60 ms, agreed for the heartbeat.    */
#define OA_STOP_ACK_TIMEOUT_MS	(500U)
#define OA_MOVE_TIMEOUT_MS	(15000U)	/* One motion_q move not finished in this
					   long: Mission Control is not answering.
					   Must stay longer than S2's AV_MOVE_CAP_MS
					   (14000) plus its AV_GAP_MS and loop slack:
					   S2 now ends moves on wheel-encoder distance
					   and stops a stalled move itself.        */

/* ------------------------------------------------- Bypass + line recovery */
/* S5 plans every leg; Mission Control drives each one (motion_q) and its
   open-loop distance / angle calibration decides how accurate they are. */
#define OA_REVERSE_CM		(15U)	/* TUNE: back off this far, then rescan.  */
/* TUNE: leave the line at this angle (spin speed is AV_TURN_PWM in
   Mission Control, unchanged). 60 = sharper than the old 45, so less
   forward travel per cm of sidestep. Every bypass distance is worked
   out from these angles in oa_state_bypass(). 1..89. */
#define OA_TURN_OUT_DEG		(60U)
/* TUNE: turn back toward the line after passing the obstacle. Leg 7
   turns back the same amount to run along the line again. Same as
   OA_TURN_OUT_DEG (60) so the return mirrors the way out. 1..90. */
#define OA_TURN_IN_DEG		(60U)
/* How far the front IR line sensors sit ahead of the drive-wheel axle,
   in cm: about 10, measured by Joe (the two sensors are about 1 cm
   apart). After the line search finds the line, the car drives this much
   further so the axle is on the line before it swings round onto it
   (see oa_state_bypass). 0 = no overrun.
   10 -> 13 (Jin, 19 Nov): Jin measured wheel centre to the front of the
   IR sensor boards with calipers on 17 Nov: 135 mm, and the 17 Nov
   bypass that rejoined the line used 135 mm.  The sensor eyes sit a
   little behind the board edge, so 13.  Joe: re-measure if unsure. */
#define OA_IR_AHEAD_CM		(13U)
/* TUNE: lining up with the line (leg 7) spins until the IR sensor on the
   turning side sees black. Expected: OA_TURN_IN_DEG (90 after a second
   search). Give up and raise NO_LINE this many degrees past that.
   90 + this must fit in 7 bits (checked in task_obstacle.c). */
#define OA_ALIGN_EXTRA_DEG	(30U)
/* Lining up, second try (Jin, 19 Nov): if the first spin did not find the
   line, spin on the same way up to this many more degrees (sharp bend in
   the bypass direction just after the obstacle). Must fit in 7 bits. */
#define OA_ALIGN_RETRY_DEG	(90U)
#define OA_DEFAULT_SIDESTEP_CM	(20U)	/* If the profile gives none (255).        */
#define OA_PASS_EXTRA_CM	(20U)	/* TUNE: obstacle depth + car length: drive
					   this far past the obstacle's front.
					   25 -> 20 (Jin, 19 Nov): a shorter
					   parallel leg after the car tests.     */
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

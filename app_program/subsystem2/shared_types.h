/*
 *	shared_types.h
 *	Subsystem 2 (Buddy 2, Sean) -- shared bit-packing and RTOS contract
 *	(Change Requirement 1, G1/G2/G10, R3).
 *
 *	From Sean's handoff of 25 Oct 2026, aligned with the code already on
 *	GitHub main. Where both define the same thing, the GitHub version is
 *	used and is included from its owner's header, never copied:
 *
 *	  Heartbeat bits      Aurelius  app_program/subsystem1/hb_bits.h
 *	  Vehicle state       Aurelius  robot_state_t, app_program/subsystem1/telemetry.h
 *	                                (4 bits, 9 states; was 3 bits / 8 states here)
 *	  Wi-Fi loss fault    Aurelius  TELEM_FAULT_WIFI_LOSS = bit 15 (was bit 0)
 *	  cmd_q message       Aurelius  comms_cmd_t (1 byte: type 3 + arg 5 bits),
 *	                                app_program/subsystem1/comms.h
 *	  obstacle_q          Joe       6-byte message, app_program/subsystem5/obstacle_msg.h
 *	                                (was a 2-byte obstacle_msg_t here)
 *	  flg_avoid bits      Joe       AVOID_EVT_*, app_program/subsystem5/subsystem5.h
 *	                                (bit 2 is AVOID_EVT_RESUME, not ABORT_FAULT)
 *	  Obstacle faults     Joe       bits 8, 9, 10 (S5_FAULT_*, subsystem5/s5_links.h)
 *	  line_err_q, barcode_q, nav  Yong Quan  track_purist/include/track_types.h, nav_cmd.h
 *	  Barcode edge overflow       Yong Quan  a COUNTER, not a fault bit (G3), so the
 *	                                         old FAULT_BIT_BARCODE_OVF is gone
 *
 *	Not taken from the handoff: gpio_isr_dispatcher.c (gpio_dispatcher_init,
 *	encoders_get_ticks). Only one owner of the GPIO interrupt may exist: in
 *	this build it is subsystem 3's drv_track.c (GP7); encoders and Joe's echo
 *	are added to that handler (TEAM.md section 3).
 *
 *	Kept from the handoff (nothing on GitHub defines them yet): the fault
 *	table layout for the other bits, motor_cmd_q, odom_q, terrain_q and
 *	the status_q snapshot.
 *
 *	Included by subsystem2.h, so the real build checks it.
 */

#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#include "../../track_purist/include/track_types.h"	/* line_err_q, barcode_q */
#include "../../track_purist/include/nav_cmd.h"		/* nav_on_line / nav_on_barcode */
#include "../subsystem1/hb_bits.h"			/* HB_* heartbeat bits */
#include "../subsystem1/telemetry.h"			/* robot_state_t, TELEM_FAULT_WIFI_LOSS */
#include "../subsystem1/comms.h"			/* comms_cmd_t (cmd_q message) */
#include "../subsystem5/obstacle_msg.h"			/* obstacle_q, 6 bytes */

/* =========================================================================
 * 1. EVENT FLAGS
 * ========================================================================= */

/* Heartbeat event flag (hb_evt): HB_MOTOR_CONTROL ... HB_TELEMETRY, from
 * hb_bits.h. Each task sets only its own bit every loop. */

/* Shared 16-bit fault event flag (fault_evt), separate from hb_evt (G2).
 * Each task sets only its own bits with tk_set_flg(); Mission Control
 * reads them every cycle and is the only task that clears them (G1, C5). */
#define FAULT_BIT_MQTT_ESTOP	(1U << 0)	/* Buddy 1: recoverable (GP20 / cmd)  */
#define FAULT_BIT_LINE_UNCAL	(1U << 1)	/* Buddy 3: line calibration failed   */
#define FAULT_BIT_COLLISION	(1U << 2)	/* Buddy 4: IMU acceleration spike    */
#define FAULT_BIT_MOTOR_STALL	(1U << 3)	/* Buddy 2: critical, PWM on, 0 ticks > 300 ms */
#define FAULT_BIT_ENC_FAULT	(1U << 4)	/* Buddy 2: critical, encoder failure */
#define FAULT_BIT_IMU_FAULT	(1U << 5)	/* Buddy 4: critical, I2C / GY-511 dead */
#define FAULT_BIT_TASK_HUNG	(1U << 6)	/* Buddy 1: critical, Safety Monitor trip */
/* bit 7 spare */
#define FAULT_BIT_US_NO_ECHO	(1U << 8)	/* Buddy 5: = S5_FAULT_NO_ECHO        */
#define FAULT_BIT_NO_SAFE_RTE	(1U << 9)	/* Buddy 5: = S5_FAULT_NO_ROUTE       */
#define FAULT_BIT_US_TIMEOUT	(1U << 10)	/* Buddy 5: = S5_FAULT_TIMEOUT        */
/* bits 11-14 spare */
#define FAULT_BIT_WIFI_LOSS	TELEM_FAULT_WIFI_LOSS	/* Buddy 1: bit 15, report-only */

#define FAULT_MASK_REPORT_ONLY	(FAULT_BIT_WIFI_LOSS)
#define FAULT_MASK_RECOVERABLE	(FAULT_BIT_MQTT_ESTOP | FAULT_BIT_LINE_UNCAL |	\
				 FAULT_BIT_COLLISION  | FAULT_BIT_US_NO_ECHO |	\
				 FAULT_BIT_NO_SAFE_RTE | FAULT_BIT_US_TIMEOUT)
#define FAULT_MASK_CRITICAL	(FAULT_BIT_MOTOR_STALL | FAULT_BIT_ENC_FAULT |	\
				 FAULT_BIT_IMU_FAULT   | FAULT_BIT_TASK_HUNG)

/* Safety Monitor E_STOP event flag (estop_evt). */
#define ESTOP_FLAG_ACTIVE	(1U << 0)

/* Obstacle handshake flag (flg_avoid): AVOID_EVT_MOTORS_STOPPED,
 * AVOID_EVT_LINE_REACQUIRED, AVOID_EVT_RESUME, owned by Joe in
 * app_program/subsystem5/subsystem5.h (included by Mission Control's .c,
 * not here, because that header needs the kernel types from app.h). */

/* Layout version (R3, G10): the same 4-bit field Aurelius's laptop decoder
 * reads (PKT_LAYOUT_VERSION in app_program/subsystem1/packets.h). */
#define PACKED_LAYOUT_VERSION	1U

/* =========================================================================
 * 2. VEHICLE STATE: robot_state_t from telemetry.h, 4 bits (0..8).
 *    Drops to 3 bits if Mission Control's final list has 8 states or fewer.
 * ========================================================================= */
#define VEH_STATE_BITS		4U

/* =========================================================================
 * 3. BIT-PACKED INTER-TASK QUEUE MESSAGES (masks and shifts)
 * ========================================================================= */

/* --- Buddy 2: motor_cmd_q (Mission Control -> Motor Control, 2 bytes) ---
 * Bits 0..2  (3 bits): mode (motor_mode_t)
 * Bits 3..7  (5 bits): speed_cms (0..20 cm/s, clamped)
 * Bits 8..15 (8 bits): param (signed steering trim -100..+100 in LINE_STEER,
 *                      or distance in cm / angle in deg for discrete moves) */
typedef uint16_t motor_cmd_msg_t;

typedef enum {
	MCMD_STOP       = 0,
	MCMD_LINE_STEER = 1,
	MCMD_FWD_DIST   = 2,
	MCMD_BWD_DIST   = 3,
	MCMD_TURN_LEFT  = 4,
	MCMD_TURN_RIGHT = 5,
	MCMD_U_TURN     = 6
} motor_mode_t;

static inline motor_cmd_msg_t motor_cmd_pack(motor_mode_t mode, uint8_t speed_cms, int8_t param)
{
	if (speed_cms > 20U) {
		speed_cms = 20U;
	}
	return (motor_cmd_msg_t)(((uint16_t)((uint16_t)mode & 0x07U)) |
				 ((uint16_t)(speed_cms & 0x1FU) << 3) |
				 ((uint16_t)((uint8_t)param) << 8));
}
static inline motor_mode_t motor_cmd_get_mode(motor_cmd_msg_t m)  { return (motor_mode_t)(m & 0x07U); }
static inline uint8_t      motor_cmd_get_speed(motor_cmd_msg_t m) { return (uint8_t)((m >> 3) & 0x1FU); }
static inline int8_t       motor_cmd_get_param(motor_cmd_msg_t m) { return (int8_t)((uint8_t)(m >> 8)); }

/* --- Buddy 2: odom_q (Motor Control -> Mission Control, 2 bytes) ---
 * Bit  0      (1 bit) : motion_done (1 = discrete move/turn finished)
 * Bits 1..5   (5 bits): current_speed_cms (0..20 cm/s)
 * Bits 6..15 (10 bits): delta_s_mm (0..1023 mm travelled in the last 20 ms) */
typedef uint16_t odom_msg_t;

static inline odom_msg_t odom_pack(bool done, uint8_t speed_cms, uint16_t ds_mm)
{
	if (speed_cms > 20U) {
		speed_cms = 20U;
	}
	if (ds_mm > 1023U) {
		ds_mm = 1023U;
	}
	return (odom_msg_t)((done ? 1U : 0U) |
			    ((uint16_t)(speed_cms & 0x1FU) << 1) |
			    ((uint16_t)(ds_mm & 0x03FFU) << 6));
}
static inline bool     odom_get_done(odom_msg_t m)  { return (m & 0x01U) != 0U; }
static inline uint8_t  odom_get_speed(odom_msg_t m) { return (uint8_t)((m >> 1) & 0x1FU); }
static inline uint16_t odom_get_ds_mm(odom_msg_t m) { return (uint16_t)((m >> 6) & 0x03FFU); }

/* --- Buddy 4: terrain_q (Terrain Monitor -> Mission Control, 2 bytes) ---
 * Bits 0..2  (3 bits): terrain_state (0=FLAT, 1=CLIMBING, 2=DESCENDING)
 * Bits 3..5  (3 bits): motion_event (imu_event_t in telemetry.h)
 * Bit  6     (1 bit) : collision_flag
 * Bits 7..13 (7 bits): peak height, 0..127 = 0.0..12.7 cm in 0.1 cm steps (G6)
 * Draft until Jin Hin confirms. */
typedef uint16_t terrain_msg_t;

static inline terrain_msg_t terrain_pack(uint8_t state, uint8_t motion, bool collision, uint8_t peak_mm)
{
	if (peak_mm > 127U) {
		peak_mm = 127U;
	}
	return (terrain_msg_t)(((uint16_t)(state & 0x07U)) |
			       ((uint16_t)(motion & 0x07U) << 3) |
			       ((uint16_t)(collision ? 1U : 0U) << 6) |
			       ((uint16_t)(peak_mm & 0x7FU) << 7));
}
static inline uint8_t terrain_get_state(terrain_msg_t m)     { return (uint8_t)(m & 0x07U); }
static inline uint8_t terrain_get_motion(terrain_msg_t m)    { return (uint8_t)((m >> 3) & 0x07U); }
static inline bool    terrain_get_collision(terrain_msg_t m) { return ((m >> 6) & 0x01U) != 0U; }
static inline uint8_t terrain_get_peak_mm(terrain_msg_t m)   { return (uint8_t)((m >> 7) & 0x7FU); }

/* --- Buddy 5: obstacle_q: OBSTACLE_MSG_SIZE (6) bytes, packed with
 *     obstacle_msg_pack() / unpacked with Joe's functions in obstacle_msg.h. */

/* --- Buddy 1: cmd_q (Telemetry -> Mission Control, 1 byte) ---
 *     comms_cmd_t from comms.h: type (comms_cmd_type_t, 3 bits) + arg
 *     (speed in cm/s, 5 bits, already capped at 20). */
typedef comms_cmd_t tele_cmd_msg_t;

/* --- Buddy 1 & 2: status_q (Mission Control -> Telemetry, 12 bytes, G1) --- */
typedef struct __attribute__((packed)) {
	uint8_t  ver_and_state;  /* bits 0..3: robot_state_t, bits 4..7: PACKED_LAYOUT_VERSION */
	uint8_t  speed_and_terr; /* bits 0..4: speed_cms (0..20), bits 5..7: terrain_state */
	uint16_t fault_mask;     /* 16-bit shared fault mask (section 1) */
	uint16_t line_err_raw;   /* latest 15-bit line_err_q value (line_err_t.packed) */
	uint8_t  barcode_raw;    /* latest 6-bit barcode_q value (barcode_t.packed) */
	uint8_t  obs_dist_cm;    /* 0..254 cm, 255 = beyond 254 cm (G4) */
	uint8_t  hump_peak_mm;   /* 0..127 = 0.0..12.7 cm (G6) */
	uint16_t total_dist_cm;  /* total distance travelled, cm */
	uint8_t  reserved;       /* pad to 12 bytes */
} status_snapshot_t;

static inline uint8_t status_pack_ver_state(robot_state_t state)
{
	return (uint8_t)(((uint8_t)state & 0x0FU) | ((PACKED_LAYOUT_VERSION & 0x0FU) << 4));
}
static inline robot_state_t status_get_state(uint8_t ver_and_state)
{
	return (robot_state_t)(ver_and_state & 0x0FU);
}
static inline uint8_t status_get_version(uint8_t ver_and_state)
{
	return (uint8_t)((ver_and_state >> 4) & 0x0FU);
}

#endif /* SHARED_TYPES_H */

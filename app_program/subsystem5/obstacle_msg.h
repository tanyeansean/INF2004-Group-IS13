/*
 *	obstacle_msg.h
 *	Subsystem 5 -- bit-packed obstacle_q message (Change Requirement 1).
 *	Public: Mission Control (S2) and the telemetry decoder unpack it.
 *
 *	Wire layout, 6 bytes = 48 bits, all fields unsigned:
 *
 *	  Byte | Bits | Field          | Range / meaning
 *	  -----+------+----------------+-----------------------------------------
 *	   0   | 0-1  | type           | 0 OBSTACLE, 1 CLEAR, 2 PLAN, 3 FAULT
 *	   0   | 2-4  | plan           | 0 STOP, 1 CONTINUE, 2 LEFT, 3 RIGHT,
 *	       |      |                | 4 REVERSE (5-7 invalid: rejected)
 *	   0   | 5-7  | state          | S5 state (8 states)
 *	   1   | 0-7  | closest_cm     | 2-254 cm, 255 = beyond 254 cm / unknown
 *	   2   | 0-7  | width_cm       | 0-254 cm, 255 = unknown
 *	   3   | 0-7  | left_clear_cm  | lateral gap left of obstacle, 255 = open
 *	   4   | 0-7  | right_clear_cm | lateral gap right of obstacle, 255 = open
 *	   5   | 0-7  | sidestep_cm    | lateral move to make, 255 = n/a
 *
 *	Packed with masks and shifts (not C bit-fields) so the byte layout is
 *	fixed by the code, not by the compiler, and the laptop can decode it.
 */

#ifndef OBSTACLE_MSG_H
#define OBSTACLE_MSG_H

#include <stdbool.h>
#include <stdint.h>

#define OBSTACLE_MSG_SIZE	(6U)

typedef enum
{
	OBS_MSG_OBSTACLE = 0,	/* Confirmed obstacle: please stop.       */
	OBS_MSG_CLEAR    = 1,	/* Path clear / bypass done: follow line. */
	OBS_MSG_PLAN     = 2,	/* Profile + avoidance plan.              */
	OBS_MSG_FAULT    = 3	/* S5 entered FAULT.                      */
} obs_msg_type_t;

typedef enum
{
	OBS_PLAN_STOP     = 0,
	OBS_PLAN_CONTINUE = 1,
	OBS_PLAN_LEFT     = 2,
	OBS_PLAN_RIGHT    = 3,
	OBS_PLAN_REVERSE  = 4
} obs_plan_t;

/* Unpacked form, used only inside one task (never shared). */
typedef struct
{
	obs_msg_type_t	type;
	obs_plan_t	plan;
	uint8_t		state;
	uint8_t		closest_cm;
	uint8_t		width_cm;
	uint8_t		left_clear_cm;
	uint8_t		right_clear_cm;
	uint8_t		sidestep_cm;
} obstacle_msg_t;

void obstacle_msg_pack(obstacle_msg_t const *p_msg,
		       uint8_t p_buf[OBSTACLE_MSG_SIZE]);

/* Returns false if a field holds an undefined code (reject the message). */
bool obstacle_msg_unpack(uint8_t const p_buf[OBSTACLE_MSG_SIZE],
			 obstacle_msg_t *p_msg);

#endif /* OBSTACLE_MSG_H */

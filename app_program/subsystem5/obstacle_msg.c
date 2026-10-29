/*
 *	obstacle_msg.c
 *	Subsystem 5 -- pack / unpack the 6-byte obstacle_q message.
 *	Pure C: no kernel calls, so it also builds in the host tests.
 */

#include "obstacle_msg.h"

#define TYPE_SHIFT	(0U)
#define TYPE_MASK	(0x03U)
#define PLAN_SHIFT	(2U)
#define PLAN_MASK	(0x07U)
#define STATE_SHIFT	(5U)
#define STATE_MASK	(0x07U)

#define PLAN_MAX	((uint8_t)OBS_PLAN_REVERSE)

void obstacle_msg_pack(obstacle_msg_t const *p_msg,
		       uint8_t p_buf[OBSTACLE_MSG_SIZE])
{
	p_buf[0] = (uint8_t)(((((uint8_t)p_msg->type) & TYPE_MASK) << TYPE_SHIFT)
			   | ((((uint8_t)p_msg->plan) & PLAN_MASK) << PLAN_SHIFT)
			   | ((p_msg->state & STATE_MASK) << STATE_SHIFT));
	p_buf[1] = p_msg->closest_cm;
	p_buf[2] = p_msg->width_cm;
	p_buf[3] = p_msg->left_clear_cm;
	p_buf[4] = p_msg->right_clear_cm;
	p_buf[5] = p_msg->sidestep_cm;
}

bool obstacle_msg_unpack(uint8_t const p_buf[OBSTACLE_MSG_SIZE],
			 obstacle_msg_t *p_msg)
{
	uint8_t const plan = (uint8_t)((p_buf[0] >> PLAN_SHIFT) & PLAN_MASK);

	p_msg->type           = (obs_msg_type_t)((p_buf[0] >> TYPE_SHIFT) & TYPE_MASK);
	p_msg->plan           = (obs_plan_t)plan;
	p_msg->state          = (uint8_t)((p_buf[0] >> STATE_SHIFT) & STATE_MASK);
	p_msg->closest_cm     = p_buf[1];
	p_msg->width_cm       = p_buf[2];
	p_msg->left_clear_cm  = p_buf[3];
	p_msg->right_clear_cm = p_buf[4];
	p_msg->sidestep_cm    = p_buf[5];

	return (plan <= PLAN_MAX);
}

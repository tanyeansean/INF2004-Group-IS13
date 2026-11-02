/*
 *	safety_logic.c
 *	Subsystem 1 -- Safety Monitor attendance logic.  See safety_logic.h.
 */

#include "safety_logic.h"

void sm_init(sm_state_t *p_s)
{
	uint32_t i;

	for (i = 0U; i < SM_TASKS; i++) {
		p_s->last_ms[i] = 0U;
	}
	p_s->armed     = 0U;
	p_s->hung      = 0U;
	p_s->ever_hung = 0U;
}

uint32_t sm_limit_ms(uint32_t task)
{
	return ((1UL << task) == HB_OBSTACLE_AVOID) ? SM_LIMIT_OBSTACLE_MS : SM_LIMIT_MS;
}

void sm_seen(sm_state_t *p_s, uint32_t seen, uint32_t now_ms)
{
	uint32_t i;

	for (i = 0U; i < SM_TASKS; i++) {
		if ((seen & (1UL << i)) != 0U) {
			p_s->last_ms[i] = now_ms;
			p_s->armed |= (1UL << i);
		}
	}
}

void sm_restart(sm_state_t *p_s, uint32_t now_ms)
{
	uint32_t i;

	for (i = 0U; i < SM_TASKS; i++) {
		p_s->last_ms[i] = now_ms;
	}
	p_s->hung = 0U;
}

uint32_t sm_check(sm_state_t *p_s, uint32_t seen, uint32_t now_ms,
		  uint32_t *p_recovered)
{
	uint32_t	newly = 0U;
	uint32_t	recovered = 0U;
	uint32_t	i;
	uint32_t	bit;

	for (i = 0U; i < SM_TASKS; i++) {
		bit = 1UL << i;
		if ((seen & bit) != 0U) {
			p_s->last_ms[i] = now_ms;
			p_s->armed |= bit;
			if ((p_s->hung & bit) != 0U) {
				p_s->hung &= ~bit;
				recovered |= bit;
			}
		} else if (((p_s->armed & bit) != 0U) && ((p_s->hung & bit) == 0U) &&
			   ((uint32_t)(now_ms - p_s->last_ms[i]) > sm_limit_ms(i))) {
			p_s->hung      |= bit;
			p_s->ever_hung |= bit;
			newly          |= bit;
		} else {
			/* on time, not watched yet, or already reported */
		}
	}
	if (p_recovered != 0) {
		*p_recovered = recovered;
	}
	return newly;
}

uint32_t sm_first(uint32_t mask)
{
	uint32_t i;

	for (i = 0U; i < SM_TASKS; i++) {
		if ((mask & (1UL << i)) != 0U) {
			return i + 1U;
		}
	}
	return 0U;
}

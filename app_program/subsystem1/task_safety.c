/*
 *	task_safety.c
 *	Subsystem 1 -- Safety Monitor task (Buddy 1, Aurelius).
 *
 *	Every task proves it is alive by setting its own HB_* bit on flg_hb
 *	(hb_bits.h).  Every SM_PERIOD_MS this task:
 *	  1. reads and clears all the bits in one call (TWF_BITCLR, TMO_POL),
 *	     so a beat that lands during the check is never lost;
 *	  2. passes them to sm_check() (safety_logic.c): a task with no beat
 *	     for longer than its limit is hung;
 *	  3. for a hung task other than Telemetry: sets FAULT_BIT_TASK_HUNG on
 *	     flg_faults (Sean's fault table) and sends ESTOP to Mission
 *	     Control on cmd_q (IF-04, the same path as the laptop's estop),
 *	     again every SM_ESTOP_REPEAT_MS while it stays hung, so a START
 *	     cannot drive on with a task hung;
 *	  4. if Mission Control or Motor Control is the hung one, the stop
 *	     cannot get through them: it stops kicking the hardware watchdog,
 *	     the chip resets within SM_WDT_TIMEOUT_MS and the motor pins go
 *	     low (brake).  The hung task's number is kept in watchdog scratch
 *	     register 0, so the next boot reports it;
 *	  5. otherwise kicks the watchdog.
 *	Telemetry late = report only: WiFi trouble must never stop the car.
 *
 *	Monitoring and the watchdog start once line calibration is done, so
 *	the 10 s console waits and the calibration countdown at boot never
 *	count.  SM_ENFORCE = 0 gives a report-only build (console, fault bits
 *	and telemetry, no E-stop, no reset) for tuning the limits.
 *
 *	Hung tasks are shown on the console and in the telemetry fault mask:
 *	bit 6 (TASK_HUNG) and bits 12-14 = the hung task's number (task_telemetry.c).
 */

#include "subsystem1.h"
#include "comms.h"
#include "safety_logic.h"
#include "../subsystem2/subsystem2.h"	/* IF-04 mbf_cmd, flg_faults, FAULT_BIT_TASK_HUNG */
#include "../subsystem3/subsystem3.h"	/* s3_is_calibrated() */

#ifndef SM_ENFORCE
#define SM_ENFORCE		1	/* 0 = report only */
#endif
#ifndef SM_WATCHDOG
#define SM_WATCHDOG		1	/* 0 = no hardware watchdog */
#endif

#define SM_PERIOD_MS		20	/* really 20-30 ms with the 10 ms tick */
#define SM_ESTOP_REPEAT_MS	200U
#define SM_WDT_TIMEOUT_MS	100U	/* NFR8: reset within 100 ms */
#define SM_RESET_REPORT_MS	10000U	/* show "reset by watchdog" this long */

/* ---- RP2040 watchdog and power-on state machine (datasheet 4.7, 2.13) ---- */
#define SM_WDT_BASE		(0x40058000U)
#define SM_WDT_CTRL		(SM_WDT_BASE + 0x00U)
#define SM_WDT_LOAD		(SM_WDT_BASE + 0x04U)
#define SM_WDT_REASON		(SM_WDT_BASE + 0x08U)
#define SM_WDT_SCRATCH0		(SM_WDT_BASE + 0x0CU)	/* 0-3 free; SDK/bootrom use 4-7 */
#define SM_WDT_TICK		(SM_WDT_BASE + 0x2CU)
#define SM_WDT_CTRL_ENABLE	(1U << 30)
#define SM_WDT_CTRL_PAUSE	((1U << 24) | (1U << 25) | (1U << 26))	/* JTAG, DBG0, DBG1 */
#define SM_WDT_REASON_TIMER	(1U << 0)
#define SM_WDT_TICK_ENABLE	(1U << 9)
#define SM_WDT_TICK_RUNNING	(1U << 10)
#define SM_XOSC_MHZ		(12U)	/* 1 us ticks from the 12 MHz clk_ref */
#define SM_PSM_WDSEL		(0x40010000U + 0x08U)
#define SM_PSM_WDSEL_NO_OSC	(0x0001FFFCU)	/* reset all but ROSC/XOSC, as the SDK */
#define SM_REG_SET		(0x2000U)
#define SM_REG_CLR		(0x3000U)
/* RP2040-E1: the watchdog counter goes down twice per tick, so 2 x us. */
#define SM_WDT_LOAD_VALUE	(SM_WDT_TIMEOUT_MS * 1000U * 2U)
#define SM_SCRATCH_MAGIC	(0x5AFE0000U)	/* + hung task number 1-7 */

EXPORT volatile UW	s1_safety_hung = 0U;	/* HB_* bits, Safety task writes */
EXPORT volatile UW	s1_safety_reset_task = 0U;	/* 1-7 after a watchdog reset */
LOCAL UW		s_reset_report_until = 0U;

LOCAL const char * const k_task_name[SM_TASKS] = {
	"Motor Control", "Mission Control", "Line Follow", "Barcode Decode",
	"Terrain Monitor", "Obstacle Avoid", "Telemetry"
};

LOCAL void print_tasks(const char *what, UW mask)
{
	UW i;

	tm_printf((UB *)"[safety] %s:", what);
	if (mask == 0U) {
		tm_printf((UB *)" none");
	}
	for (i = 0U; i < SM_TASKS; i++) {
		if ((mask & (1UL << i)) != 0U) {
			tm_printf((UB *)" %s", k_task_name[i]);
		}
	}
	tm_printf((UB *)"\n");
}

/* For task_telemetry.c: 1-7 = hung task's number (bit + 1), 0 = none.
   For SM_RESET_REPORT_MS after a watchdog reset: the task that caused it. */
EXPORT UW s1_safety_report_task(void)
{
	UW const hung = s1_safety_hung;

	if (hung != 0U) {
		return sm_first(hung);
	}
	if ((s1_safety_reset_task != 0U) && ((W)(s1_now_ms() - s_reset_report_until) < 0)) {
		return s1_safety_reset_task;
	}
	return 0U;
}

LOCAL void report_last_reset(void)
{
	UW const scratch = in_w(SM_WDT_SCRATCH0);

	if (((in_w(SM_WDT_REASON) & SM_WDT_REASON_TIMER) != 0U) &&
	    ((scratch & 0xFFFFFFF0U) == SM_SCRATCH_MAGIC) &&
	    ((scratch & 0xFU) >= 1U) && ((scratch & 0xFU) <= SM_TASKS)) {
		s_reset_report_until  = s1_now_ms() + SM_RESET_REPORT_MS;
		s1_safety_reset_task  = scratch & 0xFU;
		tm_printf((UB *)"[safety] last reset: watchdog, %s was hung\n",
			  k_task_name[s1_safety_reset_task - 1U]);
	}
	out_w(SM_WDT_SCRATCH0, 0U);
}

#if SM_WATCHDOG
LOCAL void wdt_start(void)
{
	/* 1 us ticks; the TIMER needs them too, so start them only if off. */
	if ((in_w(SM_WDT_TICK) & SM_WDT_TICK_RUNNING) == 0U) {
		out_w(SM_WDT_TICK, SM_WDT_TICK_ENABLE | SM_XOSC_MHZ);
	}
	out_w(SM_WDT_CTRL + SM_REG_CLR, SM_WDT_CTRL_ENABLE);
	out_w(SM_PSM_WDSEL + SM_REG_SET, SM_PSM_WDSEL_NO_OSC);
	out_w(SM_WDT_CTRL + SM_REG_SET, SM_WDT_CTRL_PAUSE);	/* not while debugging */
	out_w(SM_WDT_LOAD, SM_WDT_LOAD_VALUE);
	out_w(SM_WDT_CTRL + SM_REG_SET, SM_WDT_CTRL_ENABLE);
	tm_printf((UB *)"[safety] hardware watchdog on, %u ms\n", (UINT)SM_WDT_TIMEOUT_MS);
}

LOCAL void wdt_kick(void)
{
	out_w(SM_WDT_LOAD, SM_WDT_LOAD_VALUE);
}
#endif

#if SM_ENFORCE
LOCAL void send_estop(void)
{
	comms_cmd_t cmd = { (uint8_t)CMD_ESTOP, 0U };

	(void)tk_snd_mbf(mbf_cmd, &cmd, (INT)sizeof(cmd), TMO_POL);
}
#endif

EXPORT void safety_task(INT stacd, void *exinf)
{
	sm_state_t	sm;
	UINT		seen;
	UW		now;
	UW		newly;
	UW		recovered = 0U;
	UW		critical;
#if SM_ENFORCE
	UW		next_estop_ms = 0U;
#endif
	BOOL		b_watching = FALSE;
	BOOL		b_kicking = TRUE;

	(void)stacd; (void)exinf;

	sm_init(&sm);
	report_last_reset();

	for (;;) {
		tk_dly_tsk(SM_PERIOD_MS);

		seen = 0U;	/* a TMO_POL timeout leaves it untouched */
		(void)tk_wai_flg(flg_hb, HB_ALL_TASKS, TWF_ORW | TWF_BITCLR, &seen, TMO_POL);
		now = s1_now_ms();

		if (!b_watching) {
			sm_seen(&sm, seen, now);
			if (s3_is_calibrated()) {
				b_watching = TRUE;
				sm_restart(&sm, now);
				print_tasks("watching", sm.armed);
				print_tasks("no heartbeat yet (not watched until it beats)",
					    HB_ALL_TASKS & ~sm.armed);
#if SM_WATCHDOG
				wdt_start();
#endif
			}
			continue;
		}

		newly = sm_check(&sm, seen, now, &recovered);
		s1_safety_hung = sm.hung;
		if (newly != 0U) {
			print_tasks((SM_ENFORCE != 0) ? "HUNG (no heartbeat) - E-stop" : "HUNG (no heartbeat) - report only",
				    newly);
		}
		if (recovered != 0U) {
			print_tasks("heartbeat back", recovered);
		}

		critical = sm.hung & HB_STOP_ON_MISS;
		if ((newly & HB_STOP_ON_MISS) != 0U) {
			(void)tk_set_flg(flg_faults, FAULT_BIT_TASK_HUNG);	/* latched; Mission Control clears */
		}
#if SM_ENFORCE
		if ((newly & HB_STOP_ON_MISS) != 0U) {
			next_estop_ms = now;	/* E-stop at once */
		}
		if (critical != 0U) {
			if ((W)(now - next_estop_ms) >= 0) {
				send_estop();
				next_estop_ms = now + SM_ESTOP_REPEAT_MS;
			}
#if SM_WATCHDOG
			if (b_kicking && ((critical & (HB_MOTOR_CONTROL | HB_MISSION_CONTROL)) != 0U)) {
				b_kicking = FALSE;
				out_w(SM_WDT_SCRATCH0, SM_SCRATCH_MAGIC |
				      sm_first(critical & (HB_MOTOR_CONTROL | HB_MISSION_CONTROL)));
				tm_printf((UB *)"[safety] stop path hung: watchdog resets the chip within %u ms\n",
					  (UINT)SM_WDT_TIMEOUT_MS);
			}
#endif
		}
#endif
#if SM_WATCHDOG
		if (b_kicking) {
			wdt_kick();
		}
#endif
		(void)critical;
		(void)b_kicking;
	}
}

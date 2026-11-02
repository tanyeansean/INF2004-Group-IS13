/*
 *	subsystem1.h
 *	Subsystem 1 -- Telemetry (Buddy 1, Aurelius Ng Ker Fong).
 *	Student ID: 2500612.
 *
 *	Aurelius's robocar_comms v0.3.1 (packets.c, telemetry.c, and comms.h
 *	re-implemented in comms_utk.c) as a uT-Kernel task.  The network itself
 *	runs in the kernel's WiFi service task: see s1_mqtt_bridge.c.
 *
 *	Public: the heartbeat flag every task sets (hb_bits.h), and IF-05,
 *	Mission Control's data for telemetry (status_q).
 *
 *	Two tasks: telemetry_task (task_telemetry.c) and safety_task, the
 *	Safety Monitor (task_safety.c, logic in safety_logic.c).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdint.h>
#include "../app.h"
#include "hb_bits.h"
#include "telemetry.h"

#define S1_PRI_TELEMETRY	16	/* band 16-17 (TEAM.md): lowest, NFR5 */
#define S1_PRI_SAFETY		10	/* above every application task, so it
					   can always check the others (TEAM.md
					   section 4 needs a row: 5-11 is free) */

/* Heartbeat event flag (G2).  Every task sets ONLY its own HB_* bit with
   tk_set_flg() every time it wakes.  The Safety Monitor (task_safety.c)
   reads and clears all the bits every 20 ms; nobody else may clear them. */
IMPORT ID	flg_hb;

/* ---------------------------------------------------------------------
 *  IF-05: Mission Control -> Telemetry, message buffer status_q (owned
 *  by S1).  One 16-bit packed message (Change Requirement 1):
 *    bits 0-1  type: 0 STATE, 1 LINE, 2 BARCODE event, 3 JUNCTION event
 *    STATE    : bits 2-5  robot_state_t (0..8)
 *    LINE     : bits 2-3  sensor bits (bit0 left black, bit1 right black)
 *               bits 4-10 line position, signed -64..63
 *               bit  11   on_line
 *    BARCODE /
 *    JUNCTION : bits 2-6  letter 0..25, 31 = none / unreadable
 *               bits 7-9  telem_move_t (0..4)
 *  Use the s1_post_*() helpers: zero-timeout sends, a full queue drops.
 * --------------------------------------------------------------------- */
IMPORT ID	mbf_status;
IMPORT void	s1_post_state(robot_state_t state);
IMPORT void	s1_post_line(UB sensor_bits, H position, BOOL on_line);
IMPORT void	s1_post_barcode(char letter, telem_move_t move);
IMPORT void	s1_post_junction(char letter, telem_move_t move);

/* Link name for status lines ("ONLINE", ..., "off" without MQTT). */
IMPORT const char *s1_link_name(void);

/* Private to subsystem 1 */
#define S1_STATUS_STATE		(0U)
#define S1_STATUS_LINE		(1U)
#define S1_STATUS_BARCODE	(2U)
#define S1_STATUS_JUNCTION	(3U)

#define S1_MQTT_TX_MSG_MAX	(48U)	/* flags + len + sub-topic + payload */
#define S1_MQTT_RX_DATA_MAX	(8U)	/* a map is 3 bytes, a command 1     */
#define S1_RX_CMD		(1U)
#define S1_RX_MAP		(2U)

IMPORT ID		mbf_mqtt_tx;
IMPORT ID		mbf_mqtt_rx;
IMPORT volatile UW	s1_mqtt_connected;
IMPORT volatile UW	s1_mqtt_rx_dropped;
IMPORT volatile UW	s1_mqtt_pub_err;
IMPORT void		telemetry_task(INT stacd, void *exinf);
IMPORT void		safety_task(INT stacd, void *exinf);
IMPORT volatile UW	s1_safety_hung;		/* HB_* bits hung now */
IMPORT UW		s1_safety_report_task(void);	/* 0 or hung task 1-7 */
uint32_t		s1_now_ms(void);

#endif /* SUBSYSTEM1_H */

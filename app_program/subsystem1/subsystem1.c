/*
 *	subsystem1.c
 *	Subsystem 1 -- Telemetry: objects, task, IF-05 helpers.
 */

#include "subsystem1.h"
#include "comms.h"

EXPORT ID	flg_hb;		/* heartbeat flag (all tasks set, Safety Monitor reads) */
EXPORT ID	mbf_status;	/* IF-05, owned by S1 (the receiver) */
EXPORT ID	mbf_mqtt_tx;	/* Telemetry task -> WiFi task */
EXPORT ID	mbf_mqtt_rx;	/* WiFi task -> Telemetry task */

/* Message-buffer sizes: a small length header per message; 8 bytes per
   message is a safe margin (as subsystem 5 does). */
#define S1_STATUS_MSGS		(32U)
#define S1_MQTT_TX_MSGS		(16U)
#define S1_MQTT_RX_MSGS		(4U)

EXPORT uint32_t s1_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return (uint32_t)t.lo;
}

LOCAL ID make_mbf(UW msg_max, UW msgs)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)((msg_max + 8U) * msgs);
	cmbf.maxmsz = (INT)msg_max;
	return tk_cre_mbf(&cmbf);
}

LOCAL void post(UH msg)
{
	(void)tk_snd_mbf(mbf_status, &msg, (INT)sizeof(msg), TMO_POL);
}

EXPORT void s1_post_state(robot_state_t state)
{
	post((UH)(S1_STATUS_STATE | (((UH)state & 0x0FU) << 2)));
}

EXPORT void s1_post_line(UB sensor_bits, H position, BOOL on_line)
{
	if (position > 63) {
		position = 63;
	}
	if (position < -64) {
		position = -64;
	}
	post((UH)(S1_STATUS_LINE |
		  (((UH)sensor_bits & 0x03U) << 2) |
		  (((UH)position & 0x7FU) << 4) |
		  ((on_line ? 1U : 0U) << 11)));
}

LOCAL UH letter_move(UH type, char letter, telem_move_t move)
{
	UH idx = 31U;

	if ((letter >= 'A') && (letter <= 'Z')) {
		idx = (UH)(letter - 'A');
	}
	return (UH)(type | ((idx & 0x1FU) << 2) | (((UH)move & 0x07U) << 7));
}

EXPORT void s1_post_barcode(char letter, telem_move_t move)
{
	post(letter_move(S1_STATUS_BARCODE, letter, move));
}

EXPORT void s1_post_junction(char letter, telem_move_t move)
{
	post(letter_move(S1_STATUS_JUNCTION, letter, move));
}

EXPORT const char *s1_link_name(void)
{
#if TM_WIFI_CYW43 && TM_MQTT
	return comms_link_name(comms_get_link());	/* one aligned word: safe */
#else
	return "off";
#endif
}

EXPORT ER subsystem1_init(void)
{
	T_CFLG	cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_hb = tk_cre_flg(&cflg);
	if (flg_hb < E_OK) {
		return flg_hb;
	}
	mbf_status = make_mbf(sizeof(UH), S1_STATUS_MSGS);
	if (mbf_status < E_OK) {
		return mbf_status;
	}
	mbf_mqtt_tx = make_mbf(S1_MQTT_TX_MSG_MAX, S1_MQTT_TX_MSGS);
	if (mbf_mqtt_tx < E_OK) {
		return mbf_mqtt_tx;
	}
	mbf_mqtt_rx = make_mbf(1U + S1_MQTT_RX_DATA_MAX, S1_MQTT_RX_MSGS);
	return (mbf_mqtt_rx < E_OK) ? mbf_mqtt_rx : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	/* Core 0, beside the kernel's WiFi task it exchanges messages with. */
	ID tid = app_start_task(telemetry_task, S1_PRI_TELEMETRY, TP_PRC1, "telemetry_task");
	return (tid < E_OK) ? tid : E_OK;
}

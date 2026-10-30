/*
 *	s1_mqtt_bridge.c
 *	Subsystem 1 -- hooks the kernel's MQTT client (lib/liblwip/sysdepend/
 *	pico_rp2040/mqtt_utk.c) calls.  TEAM.md section 3: no student task may
 *	call lwIP/MQTT, so the network calls stay in the kernel's WiFi service
 *	task (cyw43_utk_task); these functions run in THAT task and only move
 *	bytes through two message buffers owned by subsystem 1:
 *
 *	  mbf_mqtt_tx  Telemetry task -> WiFi task   (comms_publish())
 *	  mbf_mqtt_rx  WiFi task -> Telemetry task   (cmd, cmd/barcode_map)
 *
 *	Both sides use zero-timeout calls, so neither task can block the
 *	other.  The plain C signatures (no kernel types) are what mqtt_utk.c
 *	declares as weak symbols; without this file it runs its old demo.
 */

#include <string.h>
#include "subsystem1.h"
#include "comms_config.h"
#include "packets.h"

#define TOPIC_CMD	MQTT_TOPIC_ROOT "/cmd"
#define TOPIC_MAP	MQTT_TOPIC_ROOT "/cmd/barcode_map"
#define TOPIC_STATUS	MQTT_TOPIC_ROOT "/status"

EXPORT volatile UW	s1_mqtt_connected = 0U;	/* WiFi task writes, S1 reads */
EXPORT volatile UW	s1_mqtt_rx_dropped = 0U;	/* WiFi task writes, S1 reads */
EXPORT volatile UW	s1_mqtt_pub_err = 0U;		/* WiFi task writes, S1 reads */

/* Copy a C string, always NUL-terminated; returns chars copied. */
LOCAL INT copy_str(char *p_dst, INT max, const char *p_src, INT at)
{
	while ((*p_src != '\0') && (at < (max - 1))) {
		p_dst[at++] = *p_src++;
	}
	p_dst[at] = '\0';
	return at;
}

/* Last Will: retained 1-byte status "offline" on <root>/status. */
int app_mqtt_will(char *topic, int topic_max, unsigned char *msg)
{
	UB status = 0U;

	pkt_put(&status, PKT_STATUS_ONLINE, 0U);
	pkt_put(&status, PKT_STATUS_VERSION, PKT_LAYOUT_VERSION);
	(void)copy_str(topic, topic_max, TOPIC_STATUS, 0);
	msg[0] = status;		/* never 0: the version is >= 1 */
	msg[1] = 0U;
	return 1;
}

/* Topics to subscribe to after every (re)connect. */
int app_mqtt_subscription(int index, char *topic, int topic_max)
{
	if (index == 0) {
		(void)copy_str(topic, topic_max, TOPIC_CMD, 0);
		return 1;
	}
	if (index == 1) {
		(void)copy_str(topic, topic_max, TOPIC_MAP, 0);
		return 1;
	}
	return 0;
}

/* Next message to publish, or -1 if none.  Message format in mbf_mqtt_tx:
   [flags: bit0 qos1, bit1 retain][sub-topic length][sub-topic][payload] */
int app_mqtt_tx_pop(char *topic, int topic_max, unsigned char *data, int data_max,
		    int *qos, int *retain)
{
	UB	m[S1_MQTT_TX_MSG_MAX];
	INT	n, sub_len, i, at;
	char	sub[S1_MQTT_TX_MSG_MAX];

	n = tk_rcv_mbf(mbf_mqtt_tx, m, TMO_POL);
	if (n < 2) {
		return -1;
	}
	sub_len = (INT)m[1];
	if ((2 + sub_len) > n) {
		return -1;			/* malformed: drop */
	}
	for (i = 0; i < sub_len; i++) {
		sub[i] = (char)m[2 + i];
	}
	sub[sub_len] = '\0';
	at = copy_str(topic, topic_max, MQTT_TOPIC_ROOT, 0);
	at = copy_str(topic, topic_max, "/", at);
	(void)copy_str(topic, topic_max, sub, at);

	n -= 2 + sub_len;
	if (n > data_max) {
		n = data_max;
	}
	for (i = 0; i < n; i++) {
		data[i] = m[2 + sub_len + i];
	}
	*qos    = ((m[0] & 0x01U) != 0U) ? 1 : 0;
	*retain = ((m[0] & 0x02U) != 0U) ? 1 : 0;
	return n;
}

/* A complete incoming message (all fragments). */
void app_mqtt_rx(const char *topic, const unsigned char *data, int len)
{
	UB	m[1 + S1_MQTT_RX_DATA_MAX];
	UB	kind;
	INT	i;

	if (strcmp(topic, TOPIC_CMD) == 0) {
		kind = S1_RX_CMD;
	} else if (strcmp(topic, TOPIC_MAP) == 0) {
		kind = S1_RX_MAP;
	} else {
		return;				/* not input for us */
	}
	/* Keep what fits; a longer message still reads as the wrong length. */
	if (len > (int)S1_MQTT_RX_DATA_MAX) {
		len = (int)S1_MQTT_RX_DATA_MAX;
	}
	m[0] = kind;
	for (i = 0; i < len; i++) {
		m[1 + i] = data[i];
	}
	if (tk_snd_mbf(mbf_mqtt_rx, m, 1 + len, TMO_POL) != E_OK) {
		s1_mqtt_rx_dropped++;
	}
}

void app_mqtt_link(int connected)
{
	s1_mqtt_connected = (connected != 0) ? 1U : 0U;
}

void app_mqtt_publish_failed(void)
{
	s1_mqtt_pub_err++;
}

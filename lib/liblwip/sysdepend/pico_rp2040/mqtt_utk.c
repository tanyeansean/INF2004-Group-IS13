/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 *
 *	Application hooks (team change, TEAM.md section 5, IF-06): if the
 *	application defines the app_mqtt_*() functions below (subsystem 1,
 *	app_program/subsystem1/s1_mqtt_bridge.c), this client works for it
 *	instead of the heartbeat demo: it connects with the application's
 *	Last Will and a 10 s keepalive, subscribes to its topics, hands every
 *	complete incoming message (raw bytes) to app_mqtt_rx(), and every
 *	MQTT_UTK_PUMP_MS publishes what app_mqtt_tx_pop() returns.  All of it
 *	still runs in this one task.  The hooks are weak references, so a build
 *	without them behaves exactly as before.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

/* Application mode (hooks present) */
#define MQTT_UTK_APP_KEEPALIVE_S 10     /* broker declares us dead after ~15 s */
#define MQTT_UTK_PUMP_MS        10      /* publish queued messages this often */
#define MQTT_UTK_PUMP_BURST     4       /* at most this many per pump        */
#define MQTT_UTK_TX_TOPIC_MAX   64
#define MQTT_UTK_TX_DATA_MAX    48

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

/* Optional application hooks (see the header comment). */
extern int  app_mqtt_will(char *topic, int topic_max, unsigned char *msg) __attribute__((weak));
extern int  app_mqtt_subscription(int index, char *topic, int topic_max) __attribute__((weak));
extern int  app_mqtt_tx_pop(char *topic, int topic_max, unsigned char *data, int data_max,
                            int *qos, int *retain) __attribute__((weak));
extern void app_mqtt_rx(const char *topic, const unsigned char *data, int len) __attribute__((weak));
extern void app_mqtt_link(int connected) __attribute__((weak));
extern void app_mqtt_publish_failed(void) __attribute__((weak));

#define APP_MODE()  (app_mqtt_tx_pop != NULL)

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

/* Application mode: Last Will, and one message held back if the output
 * buffer was full (retried on the next pump, so nothing is lost). */
static char		will_topic[MQTT_UTK_TX_TOPIC_MAX];
static char		will_msg[2];
static char		tx_topic[MQTT_UTK_TX_TOPIC_MAX];
static UB		tx_data[MQTT_UTK_TX_DATA_MAX];
static int		tx_len = -1;	/* -1 = nothing held */
static int		tx_qos, tx_retain;

static void heartbeat_timeout(void *arg);
static void pump_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			UB c = data[i];
			if(!APP_MODE()) {
				/* The console is a terminal -- keep raw bytes from a
				 * payload this demo did not publish from scrambling it. */
				c = (c >= 0x20 && c < 0x7f)? c: '.';
			}
			rx_data[rx_len++] = c;
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if(APP_MODE()) {
			/* Packed binary: pass the bytes on unchanged.  A truncated
			 * message still arrives with the wrong length. */
			if(app_mqtt_rx != NULL) {
				app_mqtt_rx(rx_topic, rx_data, (int)rx_len);
			}
			return;
		}
		rx_data[rx_len] = '\0';
		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
			  rx_truncated? " (truncated)": "");
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		if(!APP_MODE()) {
			tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC "\n");
		}
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		if(APP_MODE()) {
			if(app_mqtt_publish_failed != NULL) app_mqtt_publish_failed();
		} else {
			tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
		}
	}
}

static void heartbeat_timeout(void *arg)
{
	static unsigned int	count = 0;
	UB			payload[48];
	int			len;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		len = tm_sprintf(payload, (const UB*)"hb %u", count++);
		tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
		(void)mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

/* Application mode: publish what the application queued.  Uses the same
 * single timer slot the heartbeat chain uses in demo mode. */
static void pump_timeout(void *arg)
{
	int	n;

	(void)arg;

	if(!mqtt_client_is_connected(client)) {
		if(app_mqtt_link != NULL) app_mqtt_link(0);
	} else {
		for(n = 0; n < MQTT_UTK_PUMP_BURST; n++) {
			err_t err;

			if(tx_len < 0) {
				tx_len = app_mqtt_tx_pop(tx_topic, MQTT_UTK_TX_TOPIC_MAX,
							 tx_data, MQTT_UTK_TX_DATA_MAX,
							 &tx_qos, &tx_retain);
				if(tx_len < 0) {
					break;		/* queue empty */
				}
			}
			err = mqtt_publish(client, tx_topic, tx_data, (u16_t)tx_len,
					   (u8_t)tx_qos, (u8_t)tx_retain,
					   publish_done_cb, NULL);
			if(err == ERR_MEM) {
				break;		/* output buffer full: keep it, retry */
			}
			if(err != ERR_OK && app_mqtt_publish_failed != NULL) {
				app_mqtt_publish_failed();
			}
			tx_len = -1;
		}
	}
	sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks must be registered before subscribing, or the first
		 * delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		if(APP_MODE()) {
			char	topic[MQTT_UTK_TX_TOPIC_MAX];
			int	i;

			for(i = 0; (app_mqtt_subscription != NULL) &&
				   app_mqtt_subscription(i, topic, (int)sizeof(topic)); i++) {
				err = mqtt_subscribe(client, topic, 1 /* qos */,
						     subscribe_done_cb, NULL);
				if(err != ERR_OK) {
					tm_printf((UB*)"mqtt: mqtt_subscribe(%s) failed, err=%d\n",
						  topic, (int)err);
				}
			}
			if(app_mqtt_link != NULL) app_mqtt_link(1);
			sys_untimeout(pump_timeout, NULL);
			sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
			return;
		}

		/* Demo: subscribe to the same topic this client publishes to, so
		 * every heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. */
		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
	} else {
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		if(APP_MODE() && app_mqtt_link != NULL) app_mqtt_link(0);
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	if(APP_MODE()) {
		client_info.keep_alive = MQTT_UTK_APP_KEEPALIVE_S;
		/* The broker publishes this if the car vanishes. lwIP takes the
		 * will message as a C string: the application's byte is never 0. */
		if(app_mqtt_will != NULL &&
		   app_mqtt_will(will_topic, (int)sizeof(will_topic), (unsigned char*)will_msg)) {
			client_info.will_topic  = will_topic;
			client_info.will_msg    = will_msg;
			client_info.will_qos    = 1;
			client_info.will_retain = 1;
		}
	}

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}

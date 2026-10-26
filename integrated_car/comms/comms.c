/**
 * @file    comms.c
 * @brief   WiFi + MQTT connection state machine, heartbeat, command and
 *          barcode-map intake.
 *
 * Concurrency model (pico_cyw43_arch_lwip_threadsafe_background):
 *  - The WiFi driver and lwIP run in the background: from a low-priority
 *    interrupt, and also inside cyw43_arch_lwip_end() (and other cyw43 calls)
 *    when the SDK finds work pending as it releases its lock.
 *  - lwIP callbacks (mqtt_*_cb below) run in that background context.
 *    They only set flags / copy bytes into the receive ring - no printing,
 *    no publishing, no calls into other modules.
 *  - Everything else runs in comms_poll() from the main loop (the Telemetry
 *    task), and wraps every lwIP call in cyw43_arch_lwip_begin()/end().
 *    Commands and barcode maps are handled there, in arrival order.
 */
#include "comms.h"
#include "comms_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

/* ---------------------------------------------------------------------- */
/* Private definitions                                                     */
/* ---------------------------------------------------------------------- */
#define TOPIC_MAX_LEN      (64u)
#define RX_DATA_MAX        (48u)    /* bytes kept per incoming message      */
#define CMD_MAX_LEN        (48u)    /* text commands must be shorter        */
#define RX_QUEUE_LEN       (4u)     /* power of two                         */

/* Sub-topics under MQTT_TOPIC_ROOT. */
#define SUBTOPIC_CMD       "cmd"
#define SUBTOPIC_MAP       "cmd/barcode_map"       /* laptop -> car, 3 bytes */
#define SUBTOPIC_MAP_ACK   "cmd/barcode_map/ack"   /* car -> laptop, 1 byte  */

typedef enum
{
    MQTT_RES_IDLE = 0,     /* no attempt running                          */
    MQTT_RES_PENDING,      /* mqtt_client_connect() issued, waiting       */
    MQTT_RES_ACCEPTED,     /* broker accepted                             */
    MQTT_RES_LOST          /* refused, timed out, or dropped later        */
} mqtt_result_t;

typedef enum
{
    RX_CMD = 1,            /* <root>/cmd: text command                    */
    RX_MAP                 /* <root>/cmd/barcode_map: binary map          */
} rx_kind_t;

typedef struct
{
    _Alignas(4) uint8_t data[RX_DATA_MAX];  /* first and 4-byte aligned, so
                                                any reader is safe on the M0+ */
    uint8_t kind;          /* rx_kind_t                                   */
    uint8_t len;           /* bytes used in data[]                        */
} rx_msg_t;

/* ---------------------------------------------------------------------- */
/* Module state                                                            */
/* ---------------------------------------------------------------------- */
static comms_link_t          g_link = COMMS_WIFI_DOWN;
static comms_stats_t         g_stats;
static comms_cmd_handler_t   g_cmd_handler = NULL;
static comms_map_handler_t   g_map_handler = NULL;

static mqtt_client_t *       gp_mqtt = NULL;
static volatile mqtt_result_t g_mqtt_result = MQTT_RES_IDLE;

static uint32_t g_next_attempt_ms = 0u;
static uint32_t g_attempt_start_ms = 0u;
static uint32_t g_backoff_ms = RECONNECT_MIN_MS;
static uint32_t g_next_heartbeat_ms = 0u;
static uint32_t g_heartbeat_seq = 0u;
static bool     gb_was_online = false;

static char g_client_id[32];
static char g_topic_status[TOPIC_MAX_LEN];
static char g_topic_cmd[TOPIC_MAX_LEN];
static char g_topic_map[TOPIC_MAX_LEN];

/* Single-producer (callback) / single-consumer (comms_poll) ring holding
 * commands and barcode maps in arrival order. A message is written straight
 * into the slot at g_rx_head and only becomes visible to comms_poll() when
 * g_rx_head moves on, after its last fragment has arrived. */
static rx_msg_t          g_rx_queue[RX_QUEUE_LEN];
static volatile uint32_t g_rx_head = 0u;    /* written by callback  */
static volatile uint32_t g_rx_tail = 0u;    /* written by main loop */

/* Slot being filled by the current incoming message, or NULL to ignore it
 * (background callbacks only). */
static rx_msg_t * gp_rx_slot = NULL;

/* Barcode map (main-loop context only): the last map the handler accepted,
 * reported in the heartbeat, and an ack still waiting to be sent. */
static uint8_t g_map_held[COMMS_MAP_LEN];
static bool    gb_map_held = false;
static uint8_t g_map_ack = 0u;
static bool    gb_map_ack_pending = false;

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */
static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

/* Wrap-safe "has time t been reached". */
static bool deadline_passed(uint32_t t)
{
    return ((int32_t)(now_ms() - t) >= 0);
}

static bool wifi_link_up(void)
{
    return (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP);
}

static void schedule_retry(char const * p_reason)
{
    printf("[comms] %s - retry in %lu ms\n", p_reason, (unsigned long)g_backoff_ms);
    g_next_attempt_ms = now_ms() + g_backoff_ms;
    g_backoff_ms *= 2u;
    if (g_backoff_ms > RECONNECT_MAX_MS)
    {
        g_backoff_ms = RECONNECT_MAX_MS;
    }
}

static void set_link(comms_link_t link)
{
    if (link != g_link)
    {
        printf("[comms] %s -> %s\n", comms_link_name(g_link), comms_link_name(link));
        g_link = link;
    }
}

/* ---------------------------------------------------------------------- */
/* lwIP callbacks  (BACKGROUND CONTEXT - keep them tiny)                   */
/* ---------------------------------------------------------------------- */
static void mqtt_connection_cb(mqtt_client_t * p_client, void * p_arg,
                               mqtt_connection_status_t status)
{
    (void)p_client;
    (void)p_arg;
    g_mqtt_result = (status == MQTT_CONNECT_ACCEPTED) ? MQTT_RES_ACCEPTED
                                                      : MQTT_RES_LOST;
}

static void mqtt_incoming_publish_cb(void * p_arg, char const * p_topic,
                                     u32_t tot_len)
{
    uint8_t kind = 0u;      /* 0 = not input for us */

    (void)p_arg;
    gp_rx_slot = NULL;

    if (strcmp(p_topic, g_topic_cmd) == 0)
    {
        if (tot_len < CMD_MAX_LEN)
        {
            kind = (uint8_t)RX_CMD;
        }
        else
        {
            g_stats.cmds_dropped++;             /* too long to be a command */
        }
    }
    else if (strcmp(p_topic, g_topic_map) == 0)
    {
        /* Any length is taken: nav_map_apply() judges it, so the laptop
         * gets an ack even for a wrong payload. */
        kind = (uint8_t)RX_MAP;
    }
    else
    {
        /* not a topic we take input from */
    }

    if (kind != 0u)
    {
        if ((g_rx_head - g_rx_tail) < RX_QUEUE_LEN)
        {
            gp_rx_slot = &g_rx_queue[g_rx_head & (RX_QUEUE_LEN - 1u)];
            gp_rx_slot->kind = kind;
            gp_rx_slot->len  = 0u;
        }
        else
        {
            g_stats.cmds_dropped++;             /* receive queue full */
        }
    }
}

static void mqtt_incoming_data_cb(void * p_arg, u8_t const * p_data,
                                  u16_t len, u8_t flags)
{
    uint32_t room;

    (void)p_arg;

    if (gp_rx_slot == NULL)
    {
        return;
    }

    /* Keep what fits. A map longer than RX_DATA_MAX is cut short, which
     * nav_map_apply() still rejects as the wrong length. */
    room = RX_DATA_MAX - (uint32_t)gp_rx_slot->len;
    if (len > room)
    {
        len = (u16_t)room;
    }
    memcpy(&gp_rx_slot->data[gp_rx_slot->len], p_data, len);
    gp_rx_slot->len = (uint8_t)(gp_rx_slot->len + len);

    if ((flags & MQTT_DATA_FLAG_LAST) != 0u)
    {
        __compiler_memory_barrier();    /* slot complete before it is handed over */
        g_rx_head++;
        gp_rx_slot = NULL;
    }
}

static void mqtt_sub_cb(void * p_arg, err_t err)
{
    (void)p_arg;
    (void)err;   /* not reported: a failed subscribe shows up as missing acks */
}

/* ---------------------------------------------------------------------- */
/* Connection steps (MAIN LOOP CONTEXT)                                    */
/* ---------------------------------------------------------------------- */
static void wifi_start_join(void)
{
    printf("[comms] joining WiFi '%s'\n", WIFI_SSID);
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);

    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                      CYW43_AUTH_WPA2_AES_PSK) != 0)
    {
        schedule_retry("WiFi join could not start");
        set_link(COMMS_WIFI_DOWN);
        return;
    }
    g_attempt_start_ms = now_ms();
    set_link(COMMS_WIFI_JOINING);
}

static void mqtt_start_connect(void)
{
    ip_addr_t broker;
    struct mqtt_connect_client_info_t ci;
    err_t err;

    if (!ipaddr_aton(MQTT_BROKER_IP, &broker))
    {
        printf("[comms] bad MQTT_BROKER_IP '%s'\n", MQTT_BROKER_IP);
        schedule_retry("config error");
        return;
    }

    memset(&ci, 0, sizeof(ci));
    ci.client_id   = g_client_id;
    ci.keep_alive  = MQTT_KEEPALIVE_S;
    ci.will_topic  = g_topic_status;   /* broker publishes this if we vanish */
    ci.will_msg    = "offline";
    ci.will_qos    = 1u;
    ci.will_retain = 1u;

    printf("[comms] connecting to broker %s:%u\n", MQTT_BROKER_IP,
           (unsigned)MQTT_BROKER_PORT);

    g_mqtt_result = MQTT_RES_PENDING;
    cyw43_arch_lwip_begin();
    mqtt_disconnect(gp_mqtt);          /* no-op if already disconnected */
    err = mqtt_client_connect(gp_mqtt, &broker, MQTT_BROKER_PORT,
                              mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK)
    {
        g_mqtt_result = MQTT_RES_IDLE;
        schedule_retry("mqtt_client_connect failed");
        return;
    }
    g_attempt_start_ms = now_ms();
}

static void on_online(void)
{
    set_link(COMMS_ONLINE);
    g_backoff_ms = RECONNECT_MIN_MS;
    g_next_heartbeat_ms = now_ms();

    if (gb_was_online)
    {
        g_stats.reconnects++;
    }
    gb_was_online = true;

    cyw43_arch_lwip_begin();
    (void)mqtt_subscribe(gp_mqtt, g_topic_cmd, 1u, mqtt_sub_cb, NULL);
    (void)mqtt_subscribe(gp_mqtt, g_topic_map, 1u, mqtt_sub_cb, NULL);
    cyw43_arch_lwip_end();

    (void)comms_publish("status", "online", 1u, true);
    printf("[comms] ONLINE, listening on %s and %s\n", g_topic_cmd, g_topic_map);
}

static void go_offline(char const * p_reason)
{
    cyw43_arch_lwip_begin();
    mqtt_disconnect(gp_mqtt);
    cyw43_arch_lwip_end();
    g_mqtt_result = MQTT_RES_IDLE;
    schedule_retry(p_reason);
}

/* ---------------------------------------------------------------------- */
/* Heartbeat and commands                                                  */
/* ---------------------------------------------------------------------- */
static void send_heartbeat(void)
{
    char    payload[224];
    char    map_hex[2u * COMMS_MAP_LEN + 1u] = "";
    int32_t rssi = 0;

    (void)cyw43_wifi_get_rssi(&cyw43_state, &rssi);

    if (gb_map_held)
    {
        (void)snprintf(map_hex, sizeof(map_hex), "%02x%02x%02x",
                       (unsigned)g_map_held[0], (unsigned)g_map_held[1],
                       (unsigned)g_map_held[2]);
    }

    (void)snprintf(payload, sizeof(payload),
        "{\"seq\":%lu,\"up_ms\":%lu,\"rssi\":%ld,\"reconn\":%lu,"
        "\"pub_ok\":%lu,\"pub_fail\":%lu,\"cmd_rx\":%lu,\"map_rx\":%lu,"
        "\"cmd_drop\":%lu,\"map\":\"%s\"}",
        (unsigned long)g_heartbeat_seq++, (unsigned long)now_ms(), (long)rssi,
        (unsigned long)g_stats.reconnects, (unsigned long)g_stats.pub_ok,
        (unsigned long)g_stats.pub_fail, (unsigned long)g_stats.cmds_rx,
        (unsigned long)g_stats.maps_rx, (unsigned long)g_stats.cmds_dropped,
        map_hex);

    (void)comms_publish("heartbeat", payload, 0u, false);
}

static void send_ack(char const * p_cmd_text, bool b_ok, char const * p_note)
{
    char payload[112];
    (void)snprintf(payload, sizeof(payload),
                   "{\"cmd\":\"%s\",\"ok\":%d,\"note\":\"%s\"}",
                   p_cmd_text, b_ok ? 1 : 0, p_note);
    (void)comms_publish("cmd/ack", payload, 1u, false);
}

/* Parse "start", "stop", "estop", "speed 25", "reset_peak", "ping". */
static void handle_command_text(char * p_text)
{
    comms_cmd_t cmd = { CMD_UNKNOWN, 0 };
    char *      p_arg;
    uint32_t    i;

    /* lower-case and trim trailing whitespace/newline */
    for (i = 0u; p_text[i] != '\0'; i++)
    {
        p_text[i] = (char)tolower((unsigned char)p_text[i]);
    }
    while ((i > 0u) && isspace((unsigned char)p_text[i - 1u]))
    {
        p_text[--i] = '\0';
    }

    p_arg = strchr(p_text, ' ');
    if (p_arg != NULL)
    {
        *p_arg = '\0';
        p_arg++;
    }

    g_stats.cmds_rx++;
    printf("[comms] command: '%s'%s%s\n", p_text,
           (p_arg != NULL) ? " arg=" : "", (p_arg != NULL) ? p_arg : "");

    if (strcmp(p_text, "ping") == 0)
    {
        send_ack("ping", true, "pong");      /* handled here, link test */
        return;
    }
    else if (strcmp(p_text, "start") == 0)      { cmd.type = CMD_START; }
    else if (strcmp(p_text, "stop") == 0)       { cmd.type = CMD_STOP; }
    else if (strcmp(p_text, "estop") == 0)      { cmd.type = CMD_ESTOP; }
    else if (strcmp(p_text, "reset_peak") == 0) { cmd.type = CMD_RESET_PEAK; }
    else if ((strcmp(p_text, "speed") == 0) && (p_arg != NULL))
    {
        cmd.type = CMD_SET_SPEED;
        cmd.arg  = (int32_t)strtol(p_arg, NULL, 10);
    }
    else
    {
        /* fall through as CMD_UNKNOWN */
    }

    if (cmd.type == CMD_UNKNOWN)
    {
        send_ack(p_text, false, "unknown command");
    }
    else if (g_cmd_handler == NULL)
    {
        send_ack(p_text, false, "no handler registered");
    }
    else
    {
        g_cmd_handler(&cmd);
        send_ack(p_text, true, "");
    }
}

/* Send the barcode-map ack if one is waiting. Called right after a map is
 * handled and then on every online pass until it goes out (the publish can
 * fail for a moment, e.g. while lwIP's request slots are busy just after a
 * reconnect). */
static void send_pending_map_ack(void)
{
    if (gb_map_ack_pending && (g_link == COMMS_ONLINE) &&
        comms_publish_bin(SUBTOPIC_MAP_ACK, &g_map_ack, 1u, 1u, false))
    {
        gb_map_ack_pending = false;
    }
}

/* Barcode map: hand the bytes to the map handler (telemetry.c, which calls
 * nav_map_apply()) and send its result back as the 1-byte ack. If an older
 * ack is still waiting, only the newest result is sent. */
static void handle_map(uint8_t const * p_payload, uint16_t len)
{
    uint8_t ack = COMMS_MAP_ACK_NO_HANDLER;

    g_stats.maps_rx++;

    if (g_map_handler != NULL)
    {
        ack = g_map_handler(p_payload, len);
    }
    if ((ack == 0u) && (len == COMMS_MAP_LEN))
    {
        memcpy(g_map_held, p_payload, COMMS_MAP_LEN);
        gb_map_held = true;
    }
    printf("[comms] barcode map, %u byte(s) -> ack %u\n",
           (unsigned)len, (unsigned)ack);

    g_map_ack = ack;
    gb_map_ack_pending = true;
    send_pending_map_ack();
}

static void drain_rx(void)
{
    while (g_rx_tail != g_rx_head)
    {
        rx_msg_t msg;

        __compiler_memory_barrier();    /* read g_rx_head before the slot   */
        msg = g_rx_queue[g_rx_tail & (RX_QUEUE_LEN - 1u)];
        __compiler_memory_barrier();    /* copy done before freeing the slot */
        g_rx_tail++;

        if (msg.kind == (uint8_t)RX_CMD)
        {
            char text[RX_DATA_MAX + 1u];

            memcpy(text, msg.data, msg.len);
            text[msg.len] = '\0';
            handle_command_text(text);
        }
        else if (msg.kind == (uint8_t)RX_MAP)
        {
            handle_map(msg.data, msg.len);
        }
        else
        {
            /* nothing else is queued */
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */
/* ---------------------------------------------------------------------- */
bool comms_init(void)
{
    char board_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE) != 0)
    {
        printf("[comms] cyw43 init FAILED\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    /* Power-save off: lower latency for commands, small extra current. */
    (void)cyw43_wifi_pm(&cyw43_state,
                        cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20u, 1u, 1u, 1u));

    pico_get_unique_board_id_string(board_id, sizeof(board_id));
    (void)snprintf(g_client_id, sizeof(g_client_id), "robocar-%s",
                   &board_id[sizeof(board_id) - 9u]);   /* last 8 hex chars */
    (void)snprintf(g_topic_status, sizeof(g_topic_status), "%s/status",
                   MQTT_TOPIC_ROOT);
    (void)snprintf(g_topic_cmd, sizeof(g_topic_cmd), "%s/%s",
                   MQTT_TOPIC_ROOT, SUBTOPIC_CMD);
    (void)snprintf(g_topic_map, sizeof(g_topic_map), "%s/%s",
                   MQTT_TOPIC_ROOT, SUBTOPIC_MAP);

    gp_mqtt = mqtt_client_new();
    if (gp_mqtt == NULL)
    {
        printf("[comms] mqtt_client_new FAILED\n");
        return false;
    }

    cyw43_arch_lwip_begin();
    mqtt_set_inpub_callback(gp_mqtt, mqtt_incoming_publish_cb,
                            mqtt_incoming_data_cb, NULL);
    cyw43_arch_lwip_end();

    memset(&g_stats, 0, sizeof(g_stats));
    g_link = COMMS_WIFI_DOWN;
    g_next_attempt_ms = now_ms();
    printf("[comms] init ok, client id %s\n", g_client_id);
    return true;
}

void comms_poll(void)
{
    /* Handle what has arrived first, whatever the link is doing now: a
     * command or map that came in just before a drop must not wait out the
     * reconnect. (Acks need the link, so they may be lost or sent later.) */
    drain_rx();

    switch (g_link)
    {
        case COMMS_WIFI_DOWN:
            if (deadline_passed(g_next_attempt_ms))
            {
                wifi_start_join();
            }
            break;

        case COMMS_WIFI_JOINING:
        {
            int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

            if (status == CYW43_LINK_UP)
            {
                printf("[comms] WiFi up, IP %s\n",
                       ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])));
                set_link(COMMS_MQTT_CONNECTING);
                g_mqtt_result = MQTT_RES_IDLE;
                g_next_attempt_ms = now_ms();
            }
            else if ((status == CYW43_LINK_BADAUTH) || (status == CYW43_LINK_NONET) ||
                     (status == CYW43_LINK_FAIL) ||
                     deadline_passed(g_attempt_start_ms + WIFI_JOIN_TIMEOUT_MS))
            {
                schedule_retry((status == CYW43_LINK_BADAUTH) ? "wrong WiFi password" :
                               (status == CYW43_LINK_NONET)   ? "SSID not found" :
                                                                "WiFi join failed");
                set_link(COMMS_WIFI_DOWN);
            }
            else
            {
                /* still joining */
            }
            break;
        }

        case COMMS_MQTT_CONNECTING:
            if (!wifi_link_up())
            {
                go_offline("WiFi lost while connecting to broker");
                set_link(COMMS_WIFI_DOWN);
            }
            else if (g_mqtt_result == MQTT_RES_ACCEPTED)
            {
                on_online();
            }
            else if (g_mqtt_result == MQTT_RES_LOST)
            {
                go_offline("broker refused / unreachable");
            }
            else if ((g_mqtt_result == MQTT_RES_PENDING) &&
                     deadline_passed(g_attempt_start_ms + MQTT_CONNECT_TIMEOUT_MS))
            {
                go_offline("broker connect timeout");
            }
            else if ((g_mqtt_result == MQTT_RES_IDLE) && deadline_passed(g_next_attempt_ms))
            {
                mqtt_start_connect();
            }
            else
            {
                /* waiting */
            }
            break;

        case COMMS_ONLINE:
            if (!wifi_link_up())
            {
                go_offline("WiFi link lost");
                set_link(COMMS_WIFI_DOWN);
            }
            else if (g_mqtt_result != MQTT_RES_ACCEPTED)
            {
                go_offline("broker connection lost");
                set_link(COMMS_MQTT_CONNECTING);
            }
            else
            {
                send_pending_map_ack();
                if (deadline_passed(g_next_heartbeat_ms))
                {
                    g_next_heartbeat_ms += HEARTBEAT_PERIOD_MS;
                    send_heartbeat();
                }
            }
            break;

        default:
            set_link(COMMS_WIFI_DOWN);
            break;
    }
}

comms_link_t comms_get_link(void)
{
    return g_link;
}

bool comms_is_online(void)
{
    return (g_link == COMMS_ONLINE);
}

char const * comms_link_name(comms_link_t link)
{
    switch (link)
    {
        case COMMS_WIFI_DOWN:       return "WIFI_DOWN";
        case COMMS_WIFI_JOINING:    return "WIFI_JOINING";
        case COMMS_MQTT_CONNECTING: return "MQTT_CONNECTING";
        case COMMS_ONLINE:          return "ONLINE";
        default:                    return "?";
    }
}

void comms_get_stats(comms_stats_t * p_stats)
{
    if (p_stats != NULL)
    {
        *p_stats = g_stats;
    }
}

void comms_set_cmd_handler(comms_cmd_handler_t handler)
{
    g_cmd_handler = handler;
}

void comms_set_map_handler(comms_map_handler_t handler)
{
    g_map_handler = handler;
}

bool comms_get_map(uint8_t p_out[COMMS_MAP_LEN])
{
    if (!gb_map_held || (p_out == NULL))
    {
        return false;
    }
    memcpy(p_out, g_map_held, COMMS_MAP_LEN);
    return true;
}

bool comms_publish(char const * p_subtopic, char const * p_payload,
                   uint8_t qos, bool b_retain)
{
    size_t len;

    if (p_payload == NULL)
    {
        g_stats.pub_fail++;
        return false;
    }

    len = strlen(p_payload);
    if (len > UINT16_MAX)
    {
        g_stats.pub_fail++;
        return false;
    }
    return comms_publish_bin(p_subtopic, p_payload, (uint16_t)len, qos, b_retain);
}

bool comms_publish_bin(char const * p_subtopic, void const * p_data,
                       uint16_t len, uint8_t qos, bool b_retain)
{
    char  topic[TOPIC_MAX_LEN];
    err_t err;

    if ((g_link != COMMS_ONLINE) || (p_subtopic == NULL) ||
        ((p_data == NULL) && (len > 0u)))
    {
        g_stats.pub_fail++;
        return false;
    }

    (void)snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_ROOT, p_subtopic);

    cyw43_arch_lwip_begin();
    err = mqtt_publish(gp_mqtt, topic, p_data, (u16_t)len, qos,
                       b_retain ? 1u : 0u, NULL, NULL);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
    {
        g_stats.pub_ok++;
        return true;
    }
    g_stats.pub_fail++;
    return false;
}

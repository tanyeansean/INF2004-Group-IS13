/**
 * @file    comms.c
 * @brief   WiFi + MQTT connection state machine, heartbeat, command and
 *          barcode-map intake. Every payload is packed binary (packets.h).
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
#include "packets.h"

#include <stdio.h>
#include <string.h>

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
#define RX_DATA_MAX        (8u)     /* bytes kept per incoming message: a
                                       map is 3, a command 1; anything longer
                                       is cut here and still reads as the
                                       wrong length                         */
#define RX_QUEUE_LEN       (4u)     /* power of two                         */

/* Sub-topics under MQTT_TOPIC_ROOT. */
#define SUBTOPIC_STATUS    "status"                /* car -> laptop, 1 byte  */
#define SUBTOPIC_HEARTBEAT "heartbeat"             /* car -> laptop, 9 bytes */
#define SUBTOPIC_CMD       "cmd"                   /* laptop -> car, 1 byte  */
#define SUBTOPIC_CMD_ACK   "cmd/ack"               /* car -> laptop, 1 byte  */
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
    RX_CMD = 1,            /* <root>/cmd: 1-byte command                  */
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
static uint8_t  g_heartbeat_seq = 0u;       /* wraps at 256, like its field */
static bool     gb_was_online = false;

/* Last Will: the 1-byte status "offline". lwIP takes the will message as a C
 * string, which works because the byte is never 0 (the version is >= 1). */
static char g_will_msg[2];

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
 * reported in the heartbeat. */
static uint8_t g_map_held[COMMS_MAP_LEN];
static bool    gb_map_held = false;

/* Acks still waiting to go out (main-loop context only). A publish can fail
 * for a moment, e.g. while lwIP's 8 request slots are busy just after a
 * reconnect; the ack is then retried on every online pass. Only the newest
 * ack of each kind is kept. */
static uint8_t g_cmd_ack = 0u;
static bool    gb_cmd_ack_pending = false;
static uint8_t g_map_ack = 0u;
static bool    gb_map_ack_pending = false;

/* The Last Will is a C string for lwIP, so the status byte must never be 0,
 * and SPEED acks carry the speed, so it must stay below the error codes. */
_Static_assert((PKT_LAYOUT_VERSION >= 1u) && (PKT_LAYOUT_VERSION <= 15u),
               "layout version must fit 4 bits and not be 0");
_Static_assert(CMD_SPEED_MAX_CMS < PKT_ACK_BAD_ARG, "speed cap clashes with ack codes");

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
    (void)tot_len;          /* any length is taken: a wrong one gets an ack */
    gp_rx_slot = NULL;

    if (strcmp(p_topic, g_topic_cmd) == 0)
    {
        kind = (uint8_t)RX_CMD;
    }
    else if (strcmp(p_topic, g_topic_map) == 0)
    {
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

    /* Keep what fits. A message longer than RX_DATA_MAX is cut short, which
     * still reads as the wrong length. */
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
    ci.will_msg    = g_will_msg;       /* status byte "offline"              */
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

    {
        uint8_t status = 0u;

        pkt_put(&status, PKT_STATUS_ONLINE, 1u);
        pkt_put(&status, PKT_STATUS_VERSION, PKT_LAYOUT_VERSION);
        (void)comms_publish(SUBTOPIC_STATUS, &status, PKT_STATUS_LEN, 1u, true);
    }
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
/* Heartbeat: 9 packed bytes (packets.h). Counters are sent saturated. */
static void send_heartbeat(void)
{
    uint8_t  hb[PKT_HB_LEN];
    int32_t  rssi = 0;
    uint32_t minus_dbm;

    (void)cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    minus_dbm = (rssi >= 0) ? 0u : (uint32_t)(-rssi);

    memset(hb, 0, sizeof(hb));
    pkt_put(hb, PKT_HB_SEQ,        g_heartbeat_seq++);
    pkt_put(hb, PKT_HB_UPTIME,     pkt_sat(now_ms() / 1000u, PKT_HB_UPTIME));
    pkt_put(hb, PKT_HB_RSSI,       pkt_sat(minus_dbm, PKT_HB_RSSI));
    pkt_put(hb, PKT_HB_RECONNECTS, pkt_sat(g_stats.reconnects, PKT_HB_RECONNECTS));
    pkt_put(hb, PKT_HB_PUB_FAIL,   pkt_sat(g_stats.pub_fail, PKT_HB_PUB_FAIL));
    pkt_put(hb, PKT_HB_CMD_DROP,   pkt_sat(g_stats.cmds_dropped, PKT_HB_CMD_DROP));
    pkt_put(hb, PKT_HB_EV_DROP,    pkt_sat(g_stats.events_dropped, PKT_HB_EV_DROP));
    if (gb_map_held)
    {
        pkt_put(hb, PKT_HB_MAP_HELD, 1u);
        pkt_put(hb, PKT_HB_MAP, pkt_get(g_map_held, PKT_MAP_BITS));
    }

    (void)comms_publish(SUBTOPIC_HEARTBEAT, hb, PKT_HB_LEN, 0u, false);
}

static void send_pending_acks(void)
{
    if (g_link != COMMS_ONLINE)
    {
        return;
    }
    if (gb_cmd_ack_pending &&
        comms_publish(SUBTOPIC_CMD_ACK, &g_cmd_ack, PKT_ACK_LEN, 1u, false))
    {
        gb_cmd_ack_pending = false;
    }
    if (gb_map_ack_pending &&
        comms_publish(SUBTOPIC_MAP_ACK, &g_map_ack, 1u, 1u, false))
    {
        gb_map_ack_pending = false;
    }
}

static void send_cmd_ack(uint32_t opcode, uint32_t result)
{
    g_cmd_ack = 0u;
    pkt_put(&g_cmd_ack, PKT_ACK_OP, opcode);
    pkt_put(&g_cmd_ack, PKT_ACK_RESULT, result);
    gb_cmd_ack_pending = true;
    send_pending_acks();
}

/* One command byte: opcode in bits 5-7, argument in bits 0-4 (packets.h). */
static void handle_command(uint8_t const * p_data, uint8_t len)
{
    comms_cmd_t cmd = { (uint8_t)CMD_UNKNOWN, 0u };
    uint32_t    opcode;
    uint32_t    arg;
    uint32_t    result;

    g_stats.cmds_rx++;

    if (len != PKT_CMD_LEN)
    {
        printf("[comms] command of %u bytes rejected (must be 1)\n", (unsigned)len);
        send_cmd_ack(PKT_OP_UNREADABLE, PKT_ACK_BAD_LENGTH);
        return;
    }

    opcode = pkt_get(p_data, PKT_CMD_OP);
    arg    = pkt_get(p_data, PKT_CMD_ARG);
    printf("[comms] command 0x%02X (opcode %lu, arg %lu)\n", (unsigned)p_data[0],
           (unsigned long)opcode, (unsigned long)arg);

    /* Only SPEED takes an argument; anything else with one is a stray byte
     * (e.g. text typed by hand) and is refused. */
    if ((opcode != PKT_OP_SPEED) && (arg != 0u))
    {
        send_cmd_ack(opcode, PKT_ACK_BAD_ARG);
        return;
    }

    switch (opcode)
    {
        case PKT_OP_PING:
            send_cmd_ack(PKT_OP_PING, 0u);      /* handled here, link test */
            return;
        case PKT_OP_START:      cmd.type = (uint8_t)CMD_START;      break;
        case PKT_OP_STOP:       cmd.type = (uint8_t)CMD_STOP;       break;
        case PKT_OP_ESTOP:      cmd.type = (uint8_t)CMD_ESTOP;      break;
        case PKT_OP_RESET_PEAK: cmd.type = (uint8_t)CMD_RESET_PEAK; break;
        case PKT_OP_SPEED:
            cmd.type = (uint8_t)CMD_SET_SPEED;
            cmd.arg  = (uint8_t)((arg > CMD_SPEED_MAX_CMS) ? CMD_SPEED_MAX_CMS : arg) & 0x1Fu;
            break;
        default:
            send_cmd_ack(opcode, PKT_ACK_UNKNOWN_OP);
            return;
    }

    if (g_cmd_handler == NULL)
    {
        result = PKT_ACK_NO_HANDLER;
    }
    else if (!g_cmd_handler(&cmd))
    {
        result = PKT_ACK_REJECTED;              /* e.g. cmd_q full */
    }
    else
    {
        result = (cmd.type == (uint8_t)CMD_SET_SPEED) ? (uint32_t)cmd.arg : 0u;
    }
    send_cmd_ack(opcode, result);
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
    send_pending_acks();
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
            handle_command(msg.data, msg.len);
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
    (void)snprintf(g_topic_status, sizeof(g_topic_status), "%s/%s",
                   MQTT_TOPIC_ROOT, SUBTOPIC_STATUS);
    g_will_msg[0] = 0;
    pkt_put((uint8_t *)g_will_msg, PKT_STATUS_ONLINE, 0u);
    pkt_put((uint8_t *)g_will_msg, PKT_STATUS_VERSION, PKT_LAYOUT_VERSION);
    g_will_msg[1] = '\0';
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
                send_pending_acks();
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

void comms_note_event_dropped(void)
{
    g_stats.events_dropped++;
}

bool comms_publish(char const * p_subtopic, void const * p_data,
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

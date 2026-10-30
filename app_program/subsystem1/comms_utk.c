/**
 * @file    comms_utk.c
 * @brief   Buddy 1 comms.h for uT-Kernel: link state, heartbeat, 1-byte
 *          commands and acks, barcode-map intake. Every payload is packed
 *          binary (packets.h), exactly as integrated_car/comms/comms.c v0.3.1.
 *
 * What changed against comms.c: WiFi join, broker connect, reconnect and
 * every lwIP call moved to the kernel's WiFi service task (TEAM.md §3).
 * That task joins WiFi (config/wifi_secrets.h), connects with the Last
 * Will, subscribes, publishes and receives through mqtt_utk.c, which calls
 * s1_mqtt_bridge.c. This file runs only in the Telemetry task and talks to
 * the bridge through mbf_mqtt_tx / mbf_mqtt_rx.
 * handle_command(), handle_map(), the heartbeat and the ack retry are
 * Aurelius's code, unchanged apart from printf -> tm_printf.
 */
#include <string.h>
#include "subsystem1.h"
#include "comms.h"
#include "comms_config.h"
#include "packets.h"

#if TM_WIFI_CYW43 && TM_MQTT
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#endif

/* Sub-topics under MQTT_TOPIC_ROOT. */
#define SUBTOPIC_STATUS    "status"                /* car -> laptop, 1 byte  */
#define SUBTOPIC_HEARTBEAT "heartbeat"             /* car -> laptop, 9 bytes */
#define SUBTOPIC_CMD_ACK   "cmd/ack"               /* car -> laptop, 1 byte  */
#define SUBTOPIC_MAP_ACK   "cmd/barcode_map/ack"   /* car -> laptop, 1 byte  */

/* cyw43 link_status values (CYW43_LINK_* in cyw43.h) */
#define LINK_JOIN   (1)
#define LINK_NOIP   (2)
#define LINK_UP     (3)

static comms_link_t        g_link = COMMS_WIFI_DOWN;
static comms_stats_t       g_stats;
static comms_cmd_handler_t g_cmd_handler = NULL;
static comms_map_handler_t g_map_handler = NULL;

static uint32_t g_next_heartbeat_ms = 0u;
static uint8_t  g_heartbeat_seq = 0u;
static bool     gb_was_online = false;
static uint32_t g_seen_rx_dropped = 0u;

static uint8_t g_map_held[COMMS_MAP_LEN];
static bool    gb_map_held = false;

static uint8_t g_cmd_ack = 0u;
static bool    gb_cmd_ack_pending = false;
static uint8_t g_map_ack = 0u;
static bool    gb_map_ack_pending = false;

_Static_assert((PKT_LAYOUT_VERSION >= 1u) && (PKT_LAYOUT_VERSION <= 15u),
               "layout version must fit 4 bits and not be 0");
_Static_assert(CMD_SPEED_MAX_CMS < PKT_ACK_BAD_ARG, "speed cap clashes with ack codes");

static bool deadline_passed(uint32_t t)
{
    return ((int32_t)(s1_now_ms() - t) >= 0);
}

/* Link state, read from the WiFi task's status and the bridge. */
static comms_link_t read_link(void)
{
#if TM_WIFI_CYW43 && TM_MQTT
    T_CYW43_UTK_STATUS st;

    if (s1_mqtt_connected != 0u)
    {
        return COMMS_ONLINE;
    }
    cyw43_utk_get_status(&st);
    if (st.link_status == LINK_UP)
    {
        return COMMS_MQTT_CONNECTING;
    }
    if ((st.link_status == LINK_JOIN) || (st.link_status == LINK_NOIP))
    {
        return COMMS_WIFI_JOINING;
    }
#endif
    return COMMS_WIFI_DOWN;
}

static void set_link(comms_link_t link)
{
    if (link != g_link)
    {
        tm_printf((UB *)"[comms] %s -> %s\n", comms_link_name(g_link), comms_link_name(link));
        if ((g_link == COMMS_ONLINE) && gb_was_online)
        {
            g_stats.reconnects++;
        }
        g_link = link;
    }
}

static void on_online(void)
{
    uint8_t status = 0u;

    g_next_heartbeat_ms = s1_now_ms();
    gb_was_online = true;
    pkt_put(&status, PKT_STATUS_ONLINE, 1u);
    pkt_put(&status, PKT_STATUS_VERSION, PKT_LAYOUT_VERSION);
    (void)comms_publish(SUBTOPIC_STATUS, &status, PKT_STATUS_LEN, 1u, true);
    tm_printf((UB *)"[comms] ONLINE, listening on %s/cmd and %s/cmd/barcode_map\n",
              MQTT_TOPIC_ROOT, MQTT_TOPIC_ROOT);
}

/* Heartbeat: 9 packed bytes (packets.h). RSSI is not available to
 * application tasks under uT-Kernel (only the WiFi task may call cyw43),
 * so it is sent as 0. */
static void send_heartbeat(void)
{
    uint8_t hb[PKT_HB_LEN];

    memset(hb, 0, sizeof(hb));
    pkt_put(hb, PKT_HB_SEQ,        g_heartbeat_seq++);
    pkt_put(hb, PKT_HB_UPTIME,     pkt_sat(s1_now_ms() / 1000u, PKT_HB_UPTIME));
    pkt_put(hb, PKT_HB_RSSI,       0u);
    pkt_put(hb, PKT_HB_RECONNECTS, pkt_sat(g_stats.reconnects, PKT_HB_RECONNECTS));
    pkt_put(hb, PKT_HB_PUB_FAIL,   pkt_sat(g_stats.pub_fail + s1_mqtt_pub_err, PKT_HB_PUB_FAIL));
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
        tm_printf((UB *)"[comms] command of %u bytes rejected (must be 1)\n", (unsigned)len);
        send_cmd_ack(PKT_OP_UNREADABLE, PKT_ACK_BAD_LENGTH);
        return;
    }

    opcode = pkt_get(p_data, PKT_CMD_OP);
    arg    = pkt_get(p_data, PKT_CMD_ARG);
    tm_printf((UB *)"[comms] command 0x%02X (opcode %u, arg %u)\n", (unsigned)p_data[0],
              (unsigned)opcode, (unsigned)arg);

    if ((opcode != PKT_OP_SPEED) && (arg != 0u))
    {
        send_cmd_ack(opcode, PKT_ACK_BAD_ARG);
        return;
    }

    switch (opcode)
    {
        case PKT_OP_PING:
            send_cmd_ack(PKT_OP_PING, 0u);
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
        result = PKT_ACK_REJECTED;
    }
    else
    {
        result = (cmd.type == (uint8_t)CMD_SET_SPEED) ? (uint32_t)cmd.arg : 0u;
    }
    send_cmd_ack(opcode, result);
}

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
    tm_printf((UB *)"[comms] barcode map, %u byte(s) -> ack %u\n",
              (unsigned)len, (unsigned)ack);

    g_map_ack = ack;
    gb_map_ack_pending = true;
    send_pending_acks();
}

static void drain_rx(void)
{
    _Alignas(4) uint8_t m[1u + S1_MQTT_RX_DATA_MAX];
    _Alignas(4) uint8_t data[S1_MQTT_RX_DATA_MAX];
    INT n;

    while ((n = tk_rcv_mbf(mbf_mqtt_rx, m, TMO_POL)) >= 1)
    {
        uint8_t len = (uint8_t)(n - 1);

        memcpy(data, &m[1], len);       /* 4-byte aligned for the handlers */
        if (m[0] == S1_RX_CMD)
        {
            handle_command(data, len);
        }
        else if (m[0] == S1_RX_MAP)
        {
            handle_map(data, len);
        }
        else
        {
            /* nothing else is queued */
        }
    }
    /* receive-queue drops counted by the bridge (WiFi task) */
    if (s1_mqtt_rx_dropped != g_seen_rx_dropped)
    {
        g_stats.cmds_dropped += s1_mqtt_rx_dropped - g_seen_rx_dropped;
        g_seen_rx_dropped = s1_mqtt_rx_dropped;
    }
}

/* ---------------------------------------------------------------------- */
/* Public API (comms.h)                                                    */
/* ---------------------------------------------------------------------- */
bool comms_init(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    g_link = COMMS_WIFI_DOWN;
#if TM_WIFI_CYW43 && TM_MQTT
    tm_printf((UB *)"[comms] uT-Kernel WiFi task + MQTT client, topics %s/...\n",
              MQTT_TOPIC_ROOT);
    return true;
#else
    tm_printf((UB *)"[comms] built without MQTT (make NET=lwip MQTT=1) - link off\n");
    return false;
#endif
}

void comms_poll(void)
{
    comms_link_t link;

    drain_rx();

    link = read_link();
    if ((link == COMMS_ONLINE) && (g_link != COMMS_ONLINE))
    {
        set_link(link);
        on_online();
    }
    else
    {
        set_link(link);
    }

    if (g_link == COMMS_ONLINE)
    {
        send_pending_acks();
        if (deadline_passed(g_next_heartbeat_ms))
        {
            g_next_heartbeat_ms += HEARTBEAT_PERIOD_MS;
            send_heartbeat();
        }
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

/* Queue len bytes for <root>/<p_subtopic>; the WiFi task publishes them. */
bool comms_publish(char const * p_subtopic, void const * p_data,
                   uint16_t len, uint8_t qos, bool b_retain)
{
    uint8_t m[S1_MQTT_TX_MSG_MAX];
    size_t  sub_len;

    if ((g_link != COMMS_ONLINE) || (p_subtopic == NULL) ||
        ((p_data == NULL) && (len > 0u)))
    {
        g_stats.pub_fail++;
        return false;
    }
    sub_len = strlen(p_subtopic);
    if ((2u + sub_len + len) > sizeof(m))
    {
        g_stats.pub_fail++;
        return false;
    }
    m[0] = (uint8_t)(((qos != 0u) ? 0x01u : 0u) | (b_retain ? 0x02u : 0u));
    m[1] = (uint8_t)sub_len;
    memcpy(&m[2], p_subtopic, sub_len);
    if (len > 0u)
    {
        memcpy(&m[2u + sub_len], p_data, len);
    }
    if (tk_snd_mbf(mbf_mqtt_tx, m, (INT)(2u + sub_len + len), TMO_POL) != E_OK)
    {
        g_stats.pub_fail++;             /* queue full: dropped */
        return false;
    }
    g_stats.pub_ok++;
    return true;
}

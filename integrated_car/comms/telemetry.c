/**
 * @file    telemetry.c
 * @brief   Snapshot + event-queue telemetry, and the barcode-map hand-off
 *          to Buddy 3's nav_map_apply().
 *
 * Payloads: event/barcode and event/junction are 1 packed byte (Change
 * Requirement 1); the older topics are still compact JSON until the
 * data-layout table is agreed (see MQTT_DOCUMENTATION.md).
 */
#include "telemetry.h"
#include "comms.h"
#include "comms_config.h"
#include "nav_cmd.h"              /* Buddy 3: nav_map_t, nav_map_apply() */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

/* Defined by Mission Control (Buddy 2) and set up with nav_map_init() at
 * boot. Telemetry is its only writer (apply_barcode_map() below).
 * Declared here as in Yong Quan's handoff; if his nav_cmd.h already declares
 * it, delete this line. */
extern nav_map_t g_nav_map;

/* ---------------------------------------------------------------------- */
/* Private definitions                                                     */
/* ---------------------------------------------------------------------- */
#define EVENT_QUEUE_LEN     (8u)      /* power of two */
#define EVENT_PAYLOAD_LEN   (176u)
#define EVENT_TOPIC_LEN     (24u)
#define EVENTS_PER_POLL     (2u)      /* bound the work done per loop */

#define LETTER_MASK         (0x1Fu)   /* bits 0-4 of the event byte */
#define MOVE_SHIFT          (5u)      /* bits 5-7 of the event byte */

typedef struct
{
    char     subtopic[EVENT_TOPIC_LEN];
    uint16_t len;                       /* payload bytes (binary or text) */
    uint8_t  payload[EVENT_PAYLOAD_LEN];
} telem_event_t;

typedef struct
{
    robot_state_t state;
    float         speed_l_cms;
    float         speed_r_cms;
    int32_t       enc_l;
    int32_t       enc_r;
    float         distance_cm;
    uint8_t       line_bits;
    int8_t        line_pos;
    bool          b_on_line;
    float         pitch_deg;
    float         yaw_rate_dps;
    imu_event_t   imu_event;
    float         hump_peak_cm;     /* highest hump seen this run */
} telem_snapshot_t;

/* ---------------------------------------------------------------------- */
/* Module state                                                            */
/* ---------------------------------------------------------------------- */
static telem_snapshot_t g_snap;
static telem_event_t    g_events[EVENT_QUEUE_LEN];
static uint32_t         g_ev_head = 0u;
static uint32_t         g_ev_tail = 0u;
static uint32_t         g_ev_dropped = 0u;
static uint32_t         g_live_seq = 0u;
static uint32_t         g_next_live_ms = 0u;

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */
static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

/* Queue an event of len bytes; if full, the OLDEST event is overwritten. */
static void event_push_bin(char const * p_subtopic, void const * p_data,
                           uint16_t len)
{
    telem_event_t * p_ev;

    if (len > EVENT_PAYLOAD_LEN)
    {
        len = EVENT_PAYLOAD_LEN;
    }

    if ((g_ev_head - g_ev_tail) >= EVENT_QUEUE_LEN)
    {
        g_ev_tail++;
        g_ev_dropped++;
    }

    p_ev = &g_events[g_ev_head & (EVENT_QUEUE_LEN - 1u)];
    (void)snprintf(p_ev->subtopic, sizeof(p_ev->subtopic), "%s", p_subtopic);
    memcpy(p_ev->payload, p_data, len);
    p_ev->len = len;
    g_ev_head++;
}

/* Queue a text event (JSON or plain text; callers build it in a buffer of
 * at most EVENT_PAYLOAD_LEN bytes, and event_push_bin() clamps anyway). */
static void event_push(char const * p_subtopic, char const * p_payload)
{
    size_t len = strlen(p_payload);

    event_push_bin(p_subtopic, p_payload,
                   (len > EVENT_PAYLOAD_LEN) ? (uint16_t)EVENT_PAYLOAD_LEN : (uint16_t)len);
}

/* Runs inside comms_poll() - the Telemetry task, never the lwIP callback -
 * for each message on <root>/cmd/barcode_map. This is the only place in the
 * firmware that writes g_nav_map: nav_map_apply() stores the new map with
 * one aligned 32-bit write and Mission Control reads it with one 32-bit load,
 * so no lock is needed. On any error the previous map is kept. The return
 * code goes back to the laptop as the 1-byte ack. */
static uint8_t apply_barcode_map(uint8_t const * p_payload, uint16_t len)
{
    uint8_t status = (uint8_t)nav_map_apply(&g_nav_map, p_payload, len);

    if (status != NAV_MAP_OK)
    {
        printf("[telem] barcode map rejected, code %u - previous map kept\n",
               (unsigned)status);
    }
    else if (len >= 3u)
    {
        printf("[telem] barcode map applied (payload %02X %02X %02X)\n",
               (unsigned)p_payload[0], (unsigned)p_payload[1],
               (unsigned)p_payload[2]);
    }
    else
    {
        printf("[telem] barcode map applied\n");
    }
    return status;
}

static void publish_live(void)
{
    char payload[384];

    (void)snprintf(payload, sizeof(payload),
        "{\"seq\":%lu,\"t\":%lu,\"state\":\"%s\","
        "\"mot\":{\"vl\":%.1f,\"vr\":%.1f,\"el\":%ld,\"er\":%ld,\"dist\":%.1f},"
        "\"line\":{\"bits\":%u,\"pos\":%d,\"on\":%d},"
        "\"imu\":{\"pitch\":%.1f,\"yaw\":%.1f,\"ev\":\"%s\",\"peak\":%.2f},"
        "\"ev_drop\":%lu}",
        (unsigned long)g_live_seq++, (unsigned long)now_ms(),
        telem_state_name(g_snap.state),
        (double)g_snap.speed_l_cms, (double)g_snap.speed_r_cms,
        (long)g_snap.enc_l, (long)g_snap.enc_r, (double)g_snap.distance_cm,
        (unsigned)g_snap.line_bits, (int)g_snap.line_pos, g_snap.b_on_line ? 1 : 0,
        (double)g_snap.pitch_deg, (double)g_snap.yaw_rate_dps,
        telem_imu_name(g_snap.imu_event), (double)g_snap.hump_peak_cm,
        (unsigned long)g_ev_dropped);

    (void)comms_publish("telemetry/live", payload, 0u, false);
}

static void flush_events(void)
{
    uint32_t sent = 0u;

    while ((g_ev_tail != g_ev_head) && (sent < EVENTS_PER_POLL))
    {
        telem_event_t const * p_ev = &g_events[g_ev_tail & (EVENT_QUEUE_LEN - 1u)];

        if (!comms_publish_bin(p_ev->subtopic, p_ev->payload, p_ev->len, 1u, false))
        {
            break;          /* buffer full or link dropped: try next poll */
        }
        g_ev_tail++;
        sent++;
    }
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */
/* ---------------------------------------------------------------------- */
void telem_init(void)
{
    memset(&g_snap, 0, sizeof(g_snap));
    g_snap.state     = ROBOT_IDLE;
    g_snap.imu_event = IMU_STATIONARY;
    g_ev_head = 0u;
    g_ev_tail = 0u;
    g_next_live_ms = now_ms();
    comms_set_map_handler(apply_barcode_map);
}

void telem_poll(void)
{
    if (!comms_is_online())
    {
        /* Live data is not buffered: stale positions are useless.
         * Events stay queued until we are back online. */
        g_next_live_ms = now_ms();
        return;
    }

    flush_events();

    if ((int32_t)(now_ms() - g_next_live_ms) >= 0)
    {
        g_next_live_ms += TELEM_PERIOD_MS;
        publish_live();
    }
}

void telem_set_state(robot_state_t state)
{
    if (state != g_snap.state)
    {
        char payload[80];
        (void)snprintf(payload, sizeof(payload),
                       "{\"t\":%lu,\"from\":\"%s\",\"to\":\"%s\"}",
                       (unsigned long)now_ms(), telem_state_name(g_snap.state),
                       telem_state_name(state));
        g_snap.state = state;
        event_push("event/state", payload);
    }
}

void telem_set_motion(float speed_l_cms, float speed_r_cms,
                      int32_t enc_l, int32_t enc_r, float distance_cm)
{
    g_snap.speed_l_cms = speed_l_cms;
    g_snap.speed_r_cms = speed_r_cms;
    g_snap.enc_l       = enc_l;
    g_snap.enc_r       = enc_r;
    g_snap.distance_cm = distance_cm;
}

void telem_set_line(uint8_t sensor_bits, int8_t position, bool b_on_line)
{
    g_snap.line_bits = sensor_bits;
    g_snap.line_pos  = position;
    g_snap.b_on_line = b_on_line;
}

void telem_set_imu(float pitch_deg, float yaw_rate_dps, imu_event_t event)
{
    g_snap.pitch_deg    = pitch_deg;
    g_snap.yaw_rate_dps = yaw_rate_dps;
    g_snap.imu_event    = event;
}

uint8_t telem_pack_letter_move(char letter, telem_move_t move)
{
    uint32_t idx = TELEM_LETTER_NONE;
    uint32_t mv  = (uint32_t)move;

    if ((letter >= 'A') && (letter <= 'Z'))
    {
        idx = (uint32_t)(letter - 'A');
    }
    else if ((letter >= 'a') && (letter <= 'z'))
    {
        idx = (uint32_t)(letter - 'a');
    }
    else
    {
        /* not a letter: keep TELEM_LETTER_NONE */
    }

    if (mv > (uint32_t)TELEM_MOVE_UNMAPPED)
    {
        mv = (uint32_t)TELEM_MOVE_UNMAPPED;
    }
    return (uint8_t)((idx & LETTER_MASK) | (mv << MOVE_SHIFT));
}

void telem_event_barcode(char letter, telem_move_t move)
{
    uint8_t byte = telem_pack_letter_move(letter, move);
    event_push_bin("event/barcode", &byte, 1u);
}

void telem_event_junction(char letter, telem_move_t move)
{
    uint8_t byte = telem_pack_letter_move(letter, move);
    event_push_bin("event/junction", &byte, 1u);
}

void telem_event_hump(float peak_cm)
{
    char payload[96];

    if (peak_cm > g_snap.hump_peak_cm)
    {
        g_snap.hump_peak_cm = peak_cm;
    }
    (void)snprintf(payload, sizeof(payload),
                   "{\"t\":%lu,\"peak\":%.2f,\"max\":%.2f,\"dist\":%.1f}",
                   (unsigned long)now_ms(), (double)peak_cm,
                   (double)g_snap.hump_peak_cm, (double)g_snap.distance_cm);
    event_push("event/hump", payload);
}

void telem_event_obstacle(obstacle_profile_t const * p_profile)
{
    char payload[EVENT_PAYLOAD_LEN];

    if (p_profile == NULL)
    {
        return;
    }
    (void)snprintf(payload, sizeof(payload),
        "{\"t\":%lu,\"d\":%.1f,\"ang\":%.0f,\"w\":%.1f,"
        "\"cl\":%.1f,\"cr\":%.1f,\"act\":\"%s\"}",
        (unsigned long)now_ms(), (double)p_profile->distance_cm,
        (double)p_profile->angle_deg, (double)p_profile->width_cm,
        (double)p_profile->clear_left_cm, (double)p_profile->clear_right_cm,
        telem_avoid_name(p_profile->action));
    event_push("event/obstacle", payload);
}

void telem_log(char const * p_text)
{
    char payload[EVENT_PAYLOAD_LEN];

    if (p_text == NULL)
    {
        return;
    }
    /* Plain text, truncated. Avoid quotes in log text. */
    (void)snprintf(payload, sizeof(payload), "%s", p_text);
    event_push("log", payload);
}

void telem_reset_peak(void)
{
    g_snap.hump_peak_cm = 0.0f;
}

/* ---------------------------------------------------------------------- */
/* Name tables                                                             */
/* ---------------------------------------------------------------------- */
char const * telem_state_name(robot_state_t state)
{
    static char const * const names[] = {
        "IDLE", "LINE_FOLLOW", "TURNING", "ON_HUMP", "OBSTACLE_SCAN",
        "AVOIDING", "LINE_RECOVERY", "STOPPED", "ERROR"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ? names[state] : "?";
}

char const * telem_move_name(telem_move_t move)
{
    static char const * const names[] = { "LEFT", "RIGHT", "FORWARD", "UTURN", "UNMAPPED" };
    return ((uint32_t)move < (sizeof(names) / sizeof(names[0]))) ? names[move] : "?";
}

char const * telem_imu_name(imu_event_t event)
{
    static char const * const names[] = {
        "STATIONARY", "ACCELERATING", "TURNING", "CLIMBING", "DESCENDING", "IMPACT"
    };
    return ((uint32_t)event < (sizeof(names) / sizeof(names[0]))) ? names[event] : "?";
}

char const * telem_avoid_name(avoid_action_t action)
{
    static char const * const names[] = { "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE" };
    return ((uint32_t)action < (sizeof(names) / sizeof(names[0]))) ? names[action] : "?";
}

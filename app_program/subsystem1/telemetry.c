/**
 * @file    telemetry.c
 * @brief   Packed snapshot + event-queue telemetry, and the barcode-map
 *          hand-off to Buddy 3's nav_map_apply().
 *
 * Change Requirement 1: every payload is packed with masks and shifts
 * (packets.h). The live snapshot is kept packed in RAM (12 bytes) and each
 * setter writes only its own fields; only this task touches it, so the
 * read-modify-writes cannot race.
 */
#include "telemetry.h"
#include "comms.h"
#include "comms_config.h"
#include "packets.h"
#include "nav_cmd.h"              /* Buddy 3: nav_map_t, nav_map_apply() */

#include <string.h>

#include <tm/tmonitor.h>          /* uT-Kernel port: tm_printf, no stdio   */

/* uT-Kernel port: millisecond clock from subsystem1.c (tk_get_otm). */
extern uint32_t s1_now_ms(void);

/* Defined by Mission Control (Buddy 2) and set up with nav_map_init() at
 * boot. Telemetry is its only writer (apply_barcode_map() below).
 * Declared here as in Yong Quan's handoff; if his nav_cmd.h already declares
 * it, delete this line. */
extern nav_map_t g_nav_map;

/* ---------------------------------------------------------------------- */
/* Private definitions                                                     */
/* ---------------------------------------------------------------------- */
#define EVENT_QUEUE_LEN     (8u)      /* power of two */
#define EVENT_PAYLOAD_MAX   (PKT_EV_OBST_LEN)   /* largest event: 6 bytes */
#define EVENTS_PER_POLL     (2u)      /* bound the work done per loop */
#define WIFI_LOSS_SHOW_MS   (5000u)   /* keep the WiFi-loss bit this long */

typedef enum
{
    EV_STATE = 0,
    EV_BARCODE,
    EV_JUNCTION,
    EV_HUMP,
    EV_OBSTACLE,
    EV_KINDS
} ev_kind_t;

static char const * const g_ev_topics[EV_KINDS] =
{
    "event/state", "event/barcode", "event/junction", "event/hump", "event/obstacle"
};

typedef struct
{
    uint8_t kind;                       /* ev_kind_t                      */
    uint8_t len;                        /* payload bytes                  */
    uint8_t payload[EVENT_PAYLOAD_MAX];
} telem_event_t;

/* ---------------------------------------------------------------------- */
/* Module state                                                            */
/* ---------------------------------------------------------------------- */
static uint8_t       g_live[PKT_LIVE_LEN];  /* live snapshot, kept packed */
static uint8_t       g_odom[PKT_ODOM_LEN];  /* encoder counts, kept packed */
static telem_event_t g_events[EVENT_QUEUE_LEN];
static uint32_t      g_ev_head = 0u;
static uint32_t      g_ev_tail = 0u;
static uint8_t       g_live_seq = 0u;       /* wraps at 256, like its field */
static uint32_t      g_next_live_ms = 0u;
static uint32_t      g_next_odom_ms = 0u;
static bool          gb_seen_online = false;
static bool          gb_wifi_lost = false;  /* WiFi lost since it was up   */
static bool          gb_wifi_clear_armed = false;
static uint32_t      g_wifi_clear_ms = 0u;

/* Run summary: tallies kept from the same values that go out as live data
 * and events, consolidated into one message at the end of the run.       */
typedef struct
{
    bool     b_running;
    uint32_t run_no;
    uint32_t start_ms;
    uint32_t max_speed;                 /* PKT_LIVE_SPEED_* units, abs    */
    uint32_t n_barcode;
    uint32_t n_unreadable;
    uint32_t n_junction;
    uint32_t n_unmapped;
    uint32_t n_hump;
    uint32_t peak;                      /* PKT_SUM_PEAK units             */
    uint32_t n_obst;
    uint32_t obst_min;                  /* cm, 255 = none                 */
    uint32_t faults_seen;
    uint32_t reconnects_at_start;
    uint32_t ev_drop_at_start;
    uint8_t  letters[PKT_SUM_LETTERS];
    uint32_t n_letters;
} run_t;

static run_t    g_run;
static uint8_t  g_summary[PKT_SUM_LEN];
static bool     gb_summary_pending = false;

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */
static uint32_t now_ms(void)
{
    return s1_now_ms();
}

static bool reached(uint32_t t)
{
    return ((int32_t)(now_ms() - t) >= 0);
}

/* Queue an event; if full, the OLDEST event is overwritten and counted. */
static void event_push(ev_kind_t kind, uint8_t const * p_data, uint8_t len)
{
    telem_event_t * p_ev;

    if (len > EVENT_PAYLOAD_MAX)
    {
        len = EVENT_PAYLOAD_MAX;
    }

    if ((g_ev_head - g_ev_tail) >= EVENT_QUEUE_LEN)
    {
        g_ev_tail++;
        comms_note_event_dropped();
    }

    p_ev = &g_events[g_ev_head & (EVENT_QUEUE_LEN - 1u)];
    p_ev->kind = (uint8_t)kind;
    p_ev->len  = len;
    memcpy(p_ev->payload, p_data, len);
    g_ev_head++;
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
        tm_printf((UB *)"[telem] barcode map rejected, code %u - previous map kept\n",
               (unsigned)status);
    }
    else if (len >= 3u)
    {
        tm_printf((UB *)"[telem] barcode map applied (payload %02X %02X %02X)\n",
               (unsigned)p_payload[0], (unsigned)p_payload[1],
               (unsigned)p_payload[2]);
    }
    else
    {
        tm_printf((UB *)"[telem] barcode map applied\n");
    }
    return status;
}

/* 12 bytes: the packed snapshot plus a sequence number, with the WiFi-loss
 * bit added to Mission Control's fault mask. */
static void publish_live(void)
{
    uint8_t  msg[PKT_LIVE_LEN];
    uint32_t faults;

    memcpy(msg, g_live, sizeof(msg));
    pkt_put(msg, PKT_LIVE_SEQ, g_live_seq++);
    faults = pkt_get(msg, PKT_LIVE_FAULTS);
    if (gb_wifi_lost)
    {
        faults |= TELEM_FAULT_WIFI_LOSS;
    }
    pkt_put(msg, PKT_LIVE_FAULTS, faults);

    (void)comms_publish("telemetry/live", msg, PKT_LIVE_LEN, 0u, false);
}

static void flush_events(void)
{
    uint32_t sent = 0u;

    while ((g_ev_tail != g_ev_head) && (sent < EVENTS_PER_POLL))
    {
        telem_event_t const * p_ev = &g_events[g_ev_tail & (EVENT_QUEUE_LEN - 1u)];

        if (!comms_publish(g_ev_topics[p_ev->kind], p_ev->payload, p_ev->len, 1u, false))
        {
            break;          /* buffer full or link dropped: try next poll */
        }
        g_ev_tail++;
        sent++;
    }
}

/* WiFi loss is a report-only fault: set while WiFi itself is down after
 * having been up (a broker restart alone does not count), and kept for
 * WIFI_LOSS_SHOW_MS once the link is back, because live data can only reach
 * the laptop while the link is up. */
static void track_wifi_loss(comms_link_t link)
{
    if ((link == COMMS_WIFI_DOWN) || (link == COMMS_WIFI_JOINING))
    {
        if (gb_seen_online)
        {
            gb_wifi_lost = true;
        }
        gb_wifi_clear_armed = false;
        return;
    }
    if (link != COMMS_ONLINE)
    {
        return;                         /* WiFi up, broker not yet: wait */
    }

    gb_seen_online = true;
    if (gb_wifi_lost)
    {
        if (!gb_wifi_clear_armed)
        {
            g_wifi_clear_ms = now_ms() + WIFI_LOSS_SHOW_MS;
            gb_wifi_clear_armed = true;
        }
        else if (reached(g_wifi_clear_ms))
        {
            gb_wifi_lost = false;
            gb_wifi_clear_armed = false;
        }
        else
        {
            /* still showing it */
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */
/* ---------------------------------------------------------------------- */
/* A run = the car is doing something (not IDLE, STOPPED or ERROR). */
static bool is_running_state(uint32_t state)
{
    return (state != (uint32_t)ROBOT_IDLE) && (state != (uint32_t)ROBOT_STOPPED) &&
           (state != (uint32_t)ROBOT_ERROR);
}

static void run_start(void)
{
    comms_stats_t st;
    uint32_t      run_no = (g_run.run_no + 1u) & 0xFu;

    comms_get_stats(&st);
    memset(&g_run, 0, sizeof(g_run));
    g_run.b_running = true;
    g_run.run_no    = run_no;
    g_run.start_ms  = now_ms();
    g_run.obst_min  = pkt_max(PKT_SUM_OBST_MIN);
    g_run.faults_seen = pkt_get(g_live, PKT_LIVE_FAULTS);
    g_run.reconnects_at_start = st.reconnects;
    g_run.ev_drop_at_start    = st.events_dropped;
}

static void build_summary(void)
{
    comms_stats_t st;
    uint32_t      i;
    uint32_t      dur_s = (now_ms() - g_run.start_ms) / 1000u;

    comms_get_stats(&st);
    memset(g_summary, 0, sizeof(g_summary));
    pkt_put(g_summary, PKT_SUM_RUN,          g_run.run_no);
    pkt_put(g_summary, PKT_SUM_END_STATE,    pkt_get(g_live, PKT_LIVE_STATE));
    pkt_put(g_summary, PKT_SUM_DURATION,     pkt_sat(dur_s, PKT_SUM_DURATION));
    pkt_put(g_summary, PKT_SUM_DIST,         pkt_get(g_live, PKT_LIVE_DIST));
    pkt_put(g_summary, PKT_SUM_MAX_SPEED,    pkt_sat(g_run.max_speed, PKT_SUM_MAX_SPEED));
    pkt_put(g_summary, PKT_SUM_N_BARCODE,    pkt_sat(g_run.n_barcode, PKT_SUM_N_BARCODE));
    pkt_put(g_summary, PKT_SUM_N_UNREADABLE, pkt_sat(g_run.n_unreadable, PKT_SUM_N_UNREADABLE));
    pkt_put(g_summary, PKT_SUM_N_JUNCTION,   pkt_sat(g_run.n_junction, PKT_SUM_N_JUNCTION));
    pkt_put(g_summary, PKT_SUM_N_UNMAPPED,   pkt_sat(g_run.n_unmapped, PKT_SUM_N_UNMAPPED));
    pkt_put(g_summary, PKT_SUM_N_HUMP,       pkt_sat(g_run.n_hump, PKT_SUM_N_HUMP));
    pkt_put(g_summary, PKT_SUM_PEAK,         pkt_sat(g_run.peak, PKT_SUM_PEAK));
    pkt_put(g_summary, PKT_SUM_N_OBST,       pkt_sat(g_run.n_obst, PKT_SUM_N_OBST));
    pkt_put(g_summary, PKT_SUM_OBST_MIN,     g_run.obst_min);
    pkt_put(g_summary, PKT_SUM_FAULTS_SEEN,  g_run.faults_seen | (gb_wifi_lost ? TELEM_FAULT_WIFI_LOSS : 0u));
    pkt_put(g_summary, PKT_SUM_RECONNECTS,   pkt_sat(st.reconnects - g_run.reconnects_at_start, PKT_SUM_RECONNECTS));
    pkt_put(g_summary, PKT_SUM_EV_DROP,      pkt_sat(st.events_dropped - g_run.ev_drop_at_start, PKT_SUM_EV_DROP));
    for (i = 0u; i < PKT_SUM_LETTERS; i++)
    {
        pkt_field_t f = { (uint8_t)(PKT_SUM_LETTER0.pos + (5u * i)), 5u };
        pkt_put(g_summary, f, (i < g_run.n_letters) ? g_run.letters[i] : TELEM_LETTER_NONE);
    }
}

/* Retained, QoS 1: a dashboard that connects later still sees the last run. */
static void send_summary(void)
{
    if (comms_publish("run/summary", g_summary, PKT_SUM_LEN, 1u, true))
    {
        gb_summary_pending = false;
    }
    else
    {
        gb_summary_pending = true;      /* offline or buffer full: retry  */
    }
}

void telem_publish_summary(void)
{
    build_summary();
    send_summary();
}

void telem_init(void)
{
    memset(g_live, 0, sizeof(g_live));      /* state IDLE, STATIONARY, all 0 */
    memset(&g_run, 0, sizeof(g_run));
    gb_summary_pending = false;
    memset(g_odom, 0, sizeof(g_odom));
    g_ev_head = 0u;
    g_ev_tail = 0u;
    g_live_seq = 0u;
    g_next_live_ms = now_ms();
    g_next_odom_ms = g_next_live_ms;
    gb_seen_online = false;
    gb_wifi_lost = false;
    gb_wifi_clear_armed = false;
    comms_set_map_handler(apply_barcode_map);
}

void telem_poll(void)
{
    bool b_online = comms_is_online();

    track_wifi_loss(comms_get_link());

    if (!b_online)
    {
        /* Live data is not buffered: stale positions are useless.
         * Events stay queued until we are back online. */
        g_next_live_ms = now_ms();
        g_next_odom_ms = g_next_live_ms;
        return;
    }

    flush_events();
    if (gb_summary_pending)
    {
        send_summary();
    }

    if (reached(g_next_live_ms))
    {
        g_next_live_ms += TELEM_PERIOD_MS;
        publish_live();
    }
    if (reached(g_next_odom_ms))
    {
        g_next_odom_ms += ODOM_PERIOD_MS;
        (void)comms_publish("telemetry/odometry", g_odom, PKT_ODOM_LEN, 0u, false);
    }
}

void telem_set_state(robot_state_t state)
{
    uint32_t old_state = pkt_get(g_live, PKT_LIVE_STATE);
    uint32_t new_state = pkt_sat((uint32_t)state, PKT_LIVE_STATE);

    if (new_state != old_state)
    {
        uint8_t ev = 0u;

        pkt_put(&ev, PKT_EV_STATE_FROM, old_state);
        pkt_put(&ev, PKT_EV_STATE_TO, new_state);
        pkt_put(g_live, PKT_LIVE_STATE, new_state);
        event_push(EV_STATE, &ev, PKT_EV_STATE_LEN);

        if (!g_run.b_running && is_running_state(new_state))
        {
            run_start();
        }
        else if (g_run.b_running && !is_running_state(new_state))
        {
            g_run.b_running = false;        /* end of run: consolidate */
            telem_publish_summary();
        }
        else
        {
            /* still running, or still stopped */
        }
    }
}

void telem_set_motion(float speed_l_cms, float speed_r_cms,
                      int32_t enc_l, int32_t enc_r, float distance_cm)
{
    pkt_put(g_live, PKT_LIVE_SPEED_L,
            (uint32_t)pkt_scale_s(speed_l_cms, PKT_SPEED_STEP_CMS, PKT_LIVE_SPEED_L.width));
    pkt_put(g_live, PKT_LIVE_SPEED_R,
            (uint32_t)pkt_scale_s(speed_r_cms, PKT_SPEED_STEP_CMS, PKT_LIVE_SPEED_R.width));
    pkt_put(g_live, PKT_LIVE_DIST, pkt_wrap(distance_cm, PKT_DIST_STEP_CM));  /* wraps */
    {
        int32_t  l = pkt_get_signed(g_live, PKT_LIVE_SPEED_L);
        int32_t  r = pkt_get_signed(g_live, PKT_LIVE_SPEED_R);
        uint32_t m = (uint32_t)((l < 0) ? -l : l);
        uint32_t n = (uint32_t)((r < 0) ? -r : r);
        if (n > m) { m = n; }
        if (m > g_run.max_speed) { g_run.max_speed = m; }
    }
    pkt_put(g_odom, PKT_ODOM_ENC_L, (uint32_t)enc_l);   /* low 16 bits, wraps */
    pkt_put(g_odom, PKT_ODOM_ENC_R, (uint32_t)enc_r);
}

void telem_set_line(uint8_t sensor_bits, int16_t position, bool b_on_line)
{
    pkt_put(g_live, PKT_LIVE_LINE_BITS, sensor_bits);
    pkt_put(g_live, PKT_LIVE_LINE_POS,
            (uint32_t)pkt_scale_s((float)position, 1.0f, PKT_LIVE_LINE_POS.width));
    pkt_put(g_live, PKT_LIVE_ON_LINE, b_on_line ? 1u : 0u);
}

void telem_set_imu(float pitch_deg, float yaw_rate_dps, imu_event_t event)
{
    pkt_put(g_live, PKT_LIVE_PITCH,
            (uint32_t)pkt_scale_s(pitch_deg, PKT_PITCH_STEP_DEG, PKT_LIVE_PITCH.width));
    pkt_put(g_live, PKT_LIVE_YAW_RATE,
            (uint32_t)pkt_scale_s(yaw_rate_dps, PKT_YAW_STEP_DPS, PKT_LIVE_YAW_RATE.width));
    pkt_put(g_live, PKT_LIVE_MOTION, pkt_sat((uint32_t)event, PKT_LIVE_MOTION));
}

void telem_set_faults(uint16_t fault_mask)
{
    pkt_put(g_live, PKT_LIVE_FAULTS, fault_mask);
    g_run.faults_seen |= fault_mask;
}

bool telem_wifi_lost(void)
{
    return gb_wifi_lost;
}

/* Obstacle distances: whole cm, 255 = out of range. Negative (a "no echo"
 * sentinel such as -1) and NaN also mean out of range, never "touching". */
static uint32_t obstacle_cm(float cm, pkt_field_t field)
{
    if (!(cm >= 0.0f))                  /* negative or NaN */
    {
        return pkt_max(field);
    }
    return pkt_scale_u(cm, PKT_OBST_STEP_CM, pkt_max(field));
}

uint8_t telem_pack_letter_move(char letter, telem_move_t move)
{
    uint8_t  byte = 0u;
    uint32_t idx  = TELEM_LETTER_NONE;
    uint32_t mv   = (uint32_t)move;

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
    pkt_put(&byte, PKT_EV_LETTER, idx);
    pkt_put(&byte, PKT_EV_MOVE, mv);
    return byte;
}

void telem_event_barcode(char letter, telem_move_t move)
{
    uint8_t  byte = telem_pack_letter_move(letter, move);
    uint32_t idx  = pkt_get(&byte, PKT_EV_LETTER);

    if (idx == TELEM_LETTER_NONE)
    {
        g_run.n_unreadable++;
    }
    else
    {
        g_run.n_barcode++;
        if (g_run.n_letters < PKT_SUM_LETTERS)
        {
            g_run.letters[g_run.n_letters++] = (uint8_t)idx;
        }
    }
    event_push(EV_BARCODE, &byte, PKT_EV_LM_LEN);
}

void telem_event_junction(char letter, telem_move_t move)
{
    uint8_t byte = telem_pack_letter_move(letter, move);

    if (move == TELEM_MOVE_UNMAPPED)
    {
        g_run.n_unmapped++;
    }
    else
    {
        g_run.n_junction++;
    }
    event_push(EV_JUNCTION, &byte, PKT_EV_LM_LEN);
}

void telem_event_hump(float peak_cm)
{
    uint8_t  ev[PKT_EV_HUMP_LEN];
    uint32_t peak = pkt_scale_u(peak_cm, PKT_PEAK_STEP_CM, pkt_max(PKT_EV_HUMP_PEAK));

    if (peak > pkt_get(g_live, PKT_LIVE_PEAK))
    {
        pkt_put(g_live, PKT_LIVE_PEAK, peak);           /* highest this run */
    }
    g_run.n_hump++;
    if (peak > g_run.peak)
    {
        g_run.peak = peak;                  /* kept even after reset_peak */
    }

    memset(ev, 0, sizeof(ev));
    pkt_put(ev, PKT_EV_HUMP_PEAK, peak);
    pkt_put(ev, PKT_EV_HUMP_DIST, pkt_get(g_live, PKT_LIVE_DIST));
    event_push(EV_HUMP, ev, PKT_EV_HUMP_LEN);
}

void telem_event_obstacle(obstacle_profile_t const * p_profile)
{
    uint8_t ev[PKT_EV_OBST_LEN];

    if (p_profile == NULL)
    {
        return;
    }

    memset(ev, 0, sizeof(ev));
    pkt_put(ev, PKT_EV_OBST_DIST, obstacle_cm(p_profile->distance_cm, PKT_EV_OBST_DIST));
    pkt_put(ev, PKT_EV_OBST_ANGLE,
            pkt_scale_u(p_profile->angle_deg, PKT_ANGLE_STEP_DEG, pkt_max(PKT_EV_OBST_ANGLE)));
    pkt_put(ev, PKT_EV_OBST_WIDTH,
            pkt_scale_u(p_profile->width_cm, PKT_OBST_STEP_CM, pkt_max(PKT_EV_OBST_WIDTH)));
    pkt_put(ev, PKT_EV_OBST_CLEAR_L, obstacle_cm(p_profile->clear_left_cm, PKT_EV_OBST_CLEAR_L));
    pkt_put(ev, PKT_EV_OBST_CLEAR_R, obstacle_cm(p_profile->clear_right_cm, PKT_EV_OBST_CLEAR_R));
    pkt_put(ev, PKT_EV_OBST_ACTION, pkt_sat((uint32_t)p_profile->action, PKT_EV_OBST_ACTION));
    g_run.n_obst++;
    if (pkt_get(ev, PKT_EV_OBST_DIST) < g_run.obst_min)
    {
        g_run.obst_min = pkt_get(ev, PKT_EV_OBST_DIST);
    }
    event_push(EV_OBSTACLE, ev, PKT_EV_OBST_LEN);
}

void telem_reset_peak(void)
{
    pkt_put(g_live, PKT_LIVE_PEAK, 0u);
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

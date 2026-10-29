/**
 * @file    telemetry.h
 * @brief   Buddy 1 - Telemetry framework (Communication API, part 2).
 *
 * THIS is the header the other four buddies include.
 *
 * Two kinds of data:
 *  1. LIVE values (telem_set_*): cheap setters, no networking. Call as often
 *     as you like. The latest snapshot is published at a fixed rate
 *     (TELEM_PERIOD_MS) as one 12-byte message on <root>/telemetry/live;
 *     encoder counts go once a second on <root>/telemetry/odometry.
 *  2. EVENTS (telem_event_*): one-off things (barcode read, junction move,
 *     hump finished, obstacle profiled). Queued and sent once, QoS 1. If WiFi
 *     is down the last few events are kept and sent when the link comes back.
 *
 * Every payload is packed binary (Change Requirement 1). Values are passed in
 * here in normal units and packed with masks and shifts at the edge; the
 * snapshot itself is kept packed. Field ranges and steps: packets.h.
 * Debug text goes to the USB serial console (printf), never over MQTT.
 *
 * Barcode map (Buddy 3): the laptop sends which letter means LEFT, RIGHT,
 * FORWARD and U-TURN on <root>/cmd/barcode_map. telem_init() registers the
 * handler that passes it to nav_map_apply(&g_nav_map, ...), so Telemetry is
 * the only writer of g_nav_map, and only from comms_poll() (task context).
 *
 * Rules for teammates:
 *  - Call from main-loop / task context, NOT from an interrupt handler.
 *  - None of these functions block or touch the network directly.
 *  - Not thread-safe: call them from ONE task only. In the µT-Kernel build
 *    that is the Telemetry task. The snapshot is packed, so two fields can
 *    share a byte, and a setter called from another task could undo a
 *    write made at the same moment (a read-modify-write race). Other tasks,
 *    Mission Control included, pass their data (fault mask too) on status_q;
 *    the command handler already runs in the Telemetry task (comms.h).
 *
 * DRAFT CONTRACT: the enums, ranges and field lists below must be agreed with
 * the owning buddy before integration (see MQTT_DOCUMENTATION.md §8).
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Owned by Mission Control (mission state, 4 bits) ------------------- */
typedef enum
{
    ROBOT_IDLE = 0,
    ROBOT_LINE_FOLLOW,
    ROBOT_TURNING,
    ROBOT_ON_HUMP,
    ROBOT_OBSTACLE_SCAN,
    ROBOT_AVOIDING,
    ROBOT_LINE_RECOVERY,
    ROBOT_STOPPED,
    ROBOT_ERROR
} robot_state_t;

/* ---- Junction moves (Buddy 3's barcode map, reported by Mission Control) - */
/* Same order as the four 5-bit slots of the map payload. Prefixed TELEM_ so
 * nothing clashes with the names in Buddy 3's nav_cmd.h.                     */
typedef enum
{
    TELEM_MOVE_LEFT = 0,     /* map slot 0 (bits 0-4)   */
    TELEM_MOVE_RIGHT,        /* map slot 1 (bits 5-9)   */
    TELEM_MOVE_FORWARD,      /* map slot 2 (bits 10-14) */
    TELEM_MOVE_UTURN,        /* map slot 3 (bits 15-19) */
    TELEM_MOVE_UNMAPPED      /* letter is not in the current map */
} telem_move_t;

/* Letter field value for "no letter": unreadable barcode, or a junction move
 * that no barcode asked for. Real letters are 0-25 ('A'-'Z').               */
#define TELEM_LETTER_NONE   (31u)

/* Fault mask bit set by Telemetry itself: the WiFi link was lost (not just
 * the broker). Report-only (the car keeps driving); shown in live data for
 * 5 s after the link is back. Mission Control's fault table owns the other
 * bits and should keep bit 15 free (PROVISIONAL bit number); it can read the
 * same flag with telem_wifi_lost() for its shared fault mask. */
#define TELEM_FAULT_WIFI_LOSS   (1u << 15)

/* ---- Owned by Buddy 4 (IMU motion events, 3 bits) ----------------------- */
typedef enum
{
    IMU_STATIONARY = 0,
    IMU_ACCELERATING,
    IMU_TURNING,
    IMU_CLIMBING_HUMP,
    IMU_DESCENDING_HUMP,
    IMU_IMPACT
} imu_event_t;

/* ---- Owned by Buddy 5 (obstacle profile + avoidance decision) ----------- */
typedef enum
{
    AVOID_STOP = 0,
    AVOID_CONTINUE,
    AVOID_TURN_LEFT,
    AVOID_TURN_RIGHT,
    AVOID_REVERSE
} avoid_action_t;

typedef struct
{
    float          distance_cm;     /* closest point; >= 255 or < 0 = out of range */
    float          angle_deg;       /* servo angle of closest point, 0-180  */
    float          width_cm;        /* estimated obstacle width             */
    float          clear_left_cm;
    float          clear_right_cm;
    avoid_action_t action;          /* planned avoidance                    */
} obstacle_profile_t;

/* ---- Lifecycle (called once / every loop by main) ----------------------- */
void telem_init(void);              /* also registers the barcode-map handler */
void telem_poll(void);              /* publishes when due; call every loop */

/* ---- LIVE setters ------------------------------------------------------- */
void telem_set_state(robot_state_t state);               /* Mission Control */
void telem_set_motion(float speed_l_cms, float speed_r_cms,
                      int32_t enc_l, int32_t enc_r,
                      float distance_cm);                 /* Buddy 2 */
void telem_set_line(uint8_t sensor_bits, int16_t position,
                    bool b_on_line);                      /* Buddy 3, position -64..63 */
void telem_set_imu(float pitch_deg, float yaw_rate_dps,
                   imu_event_t event);                    /* Buddy 4 */
void telem_set_faults(uint16_t fault_mask);               /* Mission Control, via status_q */

/* True while the WiFi-loss fault is raised (see TELEM_FAULT_WIFI_LOSS). */
bool telem_wifi_lost(void);

/* ---- EVENTS ------------------------------------------------------------- */
/* Barcode read. letter 'A'-'Z' (anything else = unreadable); move = what the
 * current map says for that letter, or TELEM_MOVE_UNMAPPED.   Mission Control */
void telem_event_barcode(char letter, telem_move_t move);

/* Junction reached and this move made. letter = the barcode that chose it,
 * or 0 if none did.                                           Mission Control */
void telem_event_junction(char letter, telem_move_t move);

void telem_event_hump(float peak_cm);                     /* Buddy 4 */
void telem_event_obstacle(obstacle_profile_t const * p_profile); /* Buddy 5 */

/* Clears the stored highest hump peak (also via the "reset_peak" command). */
void telem_reset_peak(void);

/* The 1-byte payload of event/barcode and event/junction:
 * bits 0-4 letter (0-25, TELEM_LETTER_NONE = none), bits 5-7 move.
 * Exposed so the pack/unpack test can check it against the laptop decoder. */
uint8_t telem_pack_letter_move(char letter, telem_move_t move);

/* Name helpers, handy for your own printf debugging too. */
char const * telem_state_name(robot_state_t state);
char const * telem_move_name(telem_move_t move);
char const * telem_imu_name(imu_event_t event);
char const * telem_avoid_name(avoid_action_t action);

#endif /* TELEMETRY_H */

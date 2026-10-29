/**
 * @file    packets.h
 * @brief   Buddy 1 - packed MQTT payloads (Change Requirement 1:
 *          Minimum-Bit Data Representation). Layout version 1.
 *
 * Every MQTT payload is a bit string read little-endian: bit 0 is the least
 * significant bit of byte 0, bit 8 the LSB of byte 1, and so on. Each field
 * below is { first bit, width }. Signed fields are two's complement. Values
 * outside a field's range are clamped to the nearest end (saturate), never
 * wrapped, except the sequence numbers, the encoder counts and the distance
 * travelled, which wrap (the laptop unwraps them).
 *
 * This header is the data-layout table (G10). MQTT_DOCUMENTATION.md §3 and
 * tools/packets.py mirror it; test/run_tests.py checks this header and
 * tools/packets.py agree field by field.
 * Fields marked PROVISIONAL use ranges the owning buddy has not confirmed.
 */
#ifndef PACKETS_H
#define PACKETS_H

#include <stdbool.h>
#include <stdint.h>

#define PKT_LAYOUT_VERSION      (1u)

typedef struct
{
    uint8_t pos;            /* first bit, LSB first */
    uint8_t width;          /* number of bits       */
} pkt_field_t;

/* ---- status: 1 byte, retained, also the MQTT Last Will ------------------- */
#define PKT_STATUS_LEN          (1u)
static pkt_field_t const PKT_STATUS_ONLINE      = {  0u,  1u };  /* 1 online, 0 offline     */
static pkt_field_t const PKT_STATUS_VERSION     = {  4u,  4u };  /* PKT_LAYOUT_VERSION      */

/* ---- telemetry/live: 12 bytes, 5 Hz ------------------------------------- */
#define PKT_LIVE_LEN            (12u)
static pkt_field_t const PKT_LIVE_SEQ           = {  0u,  8u };  /* wraps at 256            */
static pkt_field_t const PKT_LIVE_STATE         = {  8u,  4u };  /* robot_state_t 0-8 (PROVISIONAL: 3 bits if <= 8 states) */
static pkt_field_t const PKT_LIVE_SPEED_L       = { 12u,  8u };  /* signed, 0.5 cm/s steps, -64.0..+63.5 (PROVISIONAL) */
static pkt_field_t const PKT_LIVE_SPEED_R       = { 20u,  8u };  /* as SPEED_L              */
static pkt_field_t const PKT_LIVE_DIST          = { 28u, 14u };  /* cm travelled, wraps every 16384 cm (PROVISIONAL) */
static pkt_field_t const PKT_LIVE_LINE_BITS     = { 42u,  3u };  /* IR sensors, 1 = black   */
static pkt_field_t const PKT_LIVE_LINE_POS      = { 45u,  7u };  /* signed line error, -64..63 */
static pkt_field_t const PKT_LIVE_ON_LINE       = { 52u,  1u };  /* 1 = line detected       */
static pkt_field_t const PKT_LIVE_PITCH         = { 53u,  8u };  /* signed, 0.5 deg steps, -64.0..+63.5 (PROVISIONAL) */
static pkt_field_t const PKT_LIVE_YAW_RATE      = { 61u,  9u };  /* signed, 1 deg/s, -256..255 (PROVISIONAL) */
static pkt_field_t const PKT_LIVE_MOTION        = { 70u,  3u };  /* imu_event_t 0-5         */
static pkt_field_t const PKT_LIVE_PEAK          = { 73u,  7u };  /* highest hump, 0.1 cm steps, 0-12.7 cm */
static pkt_field_t const PKT_LIVE_FAULTS        = { 80u, 16u };  /* fault bit mask          */

/* ---- telemetry/odometry: 4 bytes, 1 Hz ---------------------------------- */
#define PKT_ODOM_LEN            (4u)
static pkt_field_t const PKT_ODOM_ENC_L         = {  0u, 16u };  /* encoder count, low 16 bits (wraps) */
static pkt_field_t const PKT_ODOM_ENC_R         = { 16u, 16u };

/* ---- heartbeat: 9 bytes, 1 Hz ------------------------------------------- */
#define PKT_HB_LEN              (9u)
static pkt_field_t const PKT_HB_SEQ             = {  0u,  8u };  /* wraps at 256            */
static pkt_field_t const PKT_HB_UPTIME          = {  8u, 16u };  /* seconds, 0-65535        */
static pkt_field_t const PKT_HB_RSSI            = { 24u,  7u };  /* -dBm, 0-127 (-58 dBm -> 58) */
static pkt_field_t const PKT_HB_RECONNECTS      = { 31u,  4u };  /* 0-15                    */
static pkt_field_t const PKT_HB_PUB_FAIL        = { 35u,  8u };  /* 0-255                   */
static pkt_field_t const PKT_HB_CMD_DROP        = { 43u,  4u };  /* 0-15                    */
static pkt_field_t const PKT_HB_EV_DROP         = { 47u,  4u };  /* 0-15                    */
static pkt_field_t const PKT_HB_MAP_HELD        = { 51u,  1u };  /* 1 = car holds a map     */
static pkt_field_t const PKT_HB_MAP             = { 52u, 20u };  /* the 20 map bits         */

/* ---- cmd (laptop -> car) and cmd/ack: 1 byte each ----------------------- */
#define PKT_CMD_LEN             (1u)
static pkt_field_t const PKT_CMD_ARG            = {  0u,  5u };  /* SPEED: cm/s 0-31, else 0 */
static pkt_field_t const PKT_CMD_OP             = {  5u,  3u };
#define PKT_ACK_LEN             (1u)
static pkt_field_t const PKT_ACK_RESULT         = {  0u,  5u };
static pkt_field_t const PKT_ACK_OP             = {  5u,  3u };  /* opcode answered         */

/* START and SPEED have the top bit set, so no plain ASCII byte (a stray
 * "0" or "1" sent by hand) can start or speed up the car; with the argument
 * required to be 0, a text byte can at most mean PING, STOP, ESTOP or
 * RESET_PEAK. */
#define PKT_OP_PING             (0u)
#define PKT_OP_STOP             (1u)
#define PKT_OP_ESTOP            (2u)
#define PKT_OP_RESET_PEAK       (3u)
#define PKT_OP_SPEED            (4u)
#define PKT_OP_START            (5u)
#define PKT_OP_UNREADABLE       (7u)    /* in an ack: the command was not 1 byte */

/* Ack result: 0-26 = accepted (SPEED: the speed passed on), else: */
#define PKT_ACK_BAD_ARG         (27u)   /* argument must be 0 for this command */
#define PKT_ACK_REJECTED        (28u)   /* handler said no: cmd_q full, or car not ready */
#define PKT_ACK_BAD_LENGTH      (29u)   /* a command is exactly 1 byte      */
#define PKT_ACK_NO_HANDLER      (30u)
#define PKT_ACK_UNKNOWN_OP      (31u)

/* ---- events ------------------------------------------------------------- */
#define PKT_EV_STATE_LEN        (1u)
static pkt_field_t const PKT_EV_STATE_FROM      = {  0u,  4u };
static pkt_field_t const PKT_EV_STATE_TO        = {  4u,  4u };

#define PKT_EV_LM_LEN           (1u)    /* event/barcode and event/junction */
static pkt_field_t const PKT_EV_LETTER          = {  0u,  5u };  /* 0-25 = A-Z, 31 = none   */
static pkt_field_t const PKT_EV_MOVE            = {  5u,  3u };  /* 0-3 moves, 4 = UNMAPPED */

#define PKT_EV_HUMP_LEN         (3u)
static pkt_field_t const PKT_EV_HUMP_PEAK       = {  0u,  7u };  /* 0.1 cm steps, 0-12.7 cm */
static pkt_field_t const PKT_EV_HUMP_DIST       = {  7u, 14u };  /* cm travelled when found, wraps */
/* bits 21-23 reserved, 0 */

#define PKT_EV_OBST_LEN         (6u)
static pkt_field_t const PKT_EV_OBST_DIST       = {  0u,  8u };  /* cm, 255 = out of range  */
static pkt_field_t const PKT_EV_OBST_ANGLE      = {  8u,  8u };  /* servo angle, deg 0-180  */
static pkt_field_t const PKT_EV_OBST_WIDTH      = { 16u,  7u };  /* cm, 0-127 (PROVISIONAL) */
static pkt_field_t const PKT_EV_OBST_CLEAR_L    = { 23u,  8u };  /* cm, 255 = out of range  */
static pkt_field_t const PKT_EV_OBST_CLEAR_R    = { 31u,  8u };  /* cm, 255 = out of range  */
static pkt_field_t const PKT_EV_OBST_ACTION     = { 39u,  3u };  /* avoid_action_t 0-4      */
/* bits 42-47 reserved, 0 */

/* ---- cmd/barcode_map: 3 bytes (Buddy 3's layout) ------------------------ */
#define PKT_MAP_LEN             (3u)
static pkt_field_t const PKT_MAP_BITS           = {  0u, 20u };  /* LEFT, RIGHT, FORWARD, U-TURN letters */

/* ---- Scale of each quantised field (one step of the stored integer) ----- */
#define PKT_SPEED_STEP_CMS      (0.5f)
#define PKT_DIST_STEP_CM        (1.0f)
#define PKT_PITCH_STEP_DEG      (0.5f)
#define PKT_YAW_STEP_DPS        (1.0f)
#define PKT_PEAK_STEP_CM        (0.1f)
#define PKT_OBST_STEP_CM        (1.0f)
#define PKT_ANGLE_STEP_DEG      (1.0f)

/* ---- Helpers ------------------------------------------------------------ */
/* Write the low field.width bits of value into p_buf (other bits untouched). */
void     pkt_put(uint8_t * p_buf, pkt_field_t field, uint32_t value);
uint32_t pkt_get(uint8_t const * p_buf, pkt_field_t field);
int32_t  pkt_get_signed(uint8_t const * p_buf, pkt_field_t field);

/* Round value/step to the nearest whole step and clamp it to the field:
 * 0..max for pkt_scale_u, the signed range of `width` bits for pkt_scale_s.
 * NaN gives 0 (checked before dividing: the RP2040's ROM divide turns NaN
 * into infinity). */
uint32_t pkt_scale_u(float value, float step, uint32_t max);
int32_t  pkt_scale_s(float value, float step, uint8_t width);

/* Round value/step to a whole step and keep its low bits (the field wraps;
 * negative values wrap too). NaN gives 0. For counters such as distance. */
uint32_t pkt_wrap(float value, float step);

/* Largest unsigned value a field can hold, and a counter saturated to it. */
uint32_t pkt_max(pkt_field_t field);
uint32_t pkt_sat(uint32_t value, pkt_field_t field);

#endif /* PACKETS_H */

/**
 * @file    comms.h
 * @brief   Buddy 1 - WiFi + MQTT transport layer (Communication API, part 1).
 *
 * Owns: WiFi join, broker connection, automatic recovery, heartbeat,
 * and receiving commands and the barcode map from the laptop.
 * Every payload is packed binary (Change Requirement 1); the layouts are in
 * packets.h.
 *
 * Teammates normally only need comms_poll() in the main loop and
 * comms_set_cmd_handler(). Telemetry goes through telemetry.h.
 *
 * Rules:
 *  - Call everything here from the main loop, never from an ISR.
 *  - comms_poll() never blocks; call it at least every ~10 ms.
 *  - In the RTOS build only the Telemetry task calls these. Give that task a
 *    generous stack: the SDK runs pending WiFi/lwIP work (including the MQTT
 *    receive callbacks) inside its cyw43_arch_lwip_end() calls.
 */
#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

/** Connection state. */
typedef enum
{
    COMMS_WIFI_DOWN = 0,     /**< not associated, waiting to retry        */
    COMMS_WIFI_JOINING,      /**< association + DHCP in progress          */
    COMMS_MQTT_CONNECTING,   /**< WiFi up, connecting to the broker       */
    COMMS_ONLINE             /**< WiFi + MQTT up, telemetry flowing       */
} comms_link_t;

/** Commands the laptop can send on <root>/cmd (1 byte, see packets.h). */
typedef enum
{
    CMD_START = 0,           /**< opcode 5                                */
    CMD_STOP,                /**< opcode 1 - controlled stop              */
    CMD_ESTOP,               /**< opcode 2 - emergency stop               */
    CMD_SET_SPEED,           /**< opcode 4 - arg = target speed, cm/s,
                                  already capped at CMD_SPEED_MAX_CMS     */
    CMD_RESET_PEAK,          /**< opcode 3 - clear stored hump peak       */
    CMD_UNKNOWN
} comms_cmd_type_t;

/** One command in 1 byte (C bit-fields), the size of a cmd_q message. */
typedef struct
{
    uint8_t type : 3;        /**< comms_cmd_type_t                        */
    uint8_t arg  : 5;        /**< CMD_SET_SPEED: cm/s 0-20, else 0        */
} comms_cmd_t;

/**
 * Called from comms_poll() for every valid command, i.e. in the Telemetry
 * task (main loop in the demo), never from an interrupt.
 * Return true if the command was accepted (queued to Mission Control), false
 * if it could not be (e.g. cmd_q full, or the car is not ready yet). The ack
 * tells the laptop which.
 * Because it runs in the Telemetry task, the handler may call telem_*()
 * directly, e.g. telem_reset_peak() for CMD_RESET_PEAK.
 */
typedef bool (*comms_cmd_handler_t)(comms_cmd_t const * p_cmd);

/**
 * Called from comms_poll() (main-loop / Telemetry task context, never from
 * the lwIP callback) for every message on <root>/cmd/barcode_map.
 * p_payload (4-byte aligned) holds the bytes as received; len may be 0, or
 * anything other than 3 if the laptop sent the wrong thing (at most 8 bytes
 * are kept, so a longer payload still reads as the wrong length).
 * Return 0 if the map was applied, otherwise an error code; the value is sent
 * back as the 1-byte ack on <root>/cmd/barcode_map/ack.
 * telemetry.c registers the handler that calls nav_map_apply().
 */
typedef uint8_t (*comms_map_handler_t)(uint8_t const * p_payload, uint16_t len);

/** Size of a barcode-map payload. */
#define COMMS_MAP_LEN             (3u)

/** Ack byte sent if a map arrives and no map handler is registered. */
#define COMMS_MAP_ACK_NO_HANDLER  (0xFFu)

/** Link counters (full width here; the heartbeat sends them saturated). */
typedef struct
{
    uint32_t reconnects;     /**< times the link dropped after being online */
    uint32_t pub_ok;
    uint32_t pub_fail;       /**< publish refused (buffer full / offline)   */
    uint32_t cmds_rx;        /**< commands received                         */
    uint32_t maps_rx;        /**< barcode-map messages received             */
    uint32_t cmds_dropped;   /**< receive queue full or message too long    */
    uint32_t events_dropped; /**< telemetry events lost to a full queue     */
} comms_stats_t;

/** Initialise the WiFi chip and MQTT client. Returns false if the chip fails. */
bool         comms_init(void);

/** Run the connection state machine, heartbeat and command dispatch. */
void         comms_poll(void);

comms_link_t comms_get_link(void);
bool         comms_is_online(void);
char const * comms_link_name(comms_link_t link);
void         comms_get_stats(comms_stats_t * p_stats);

/** Register Mission Control's command handler (one handler). */
void         comms_set_cmd_handler(comms_cmd_handler_t handler);

/** Register the barcode-map handler (one handler; telem_init() does this). */
void         comms_set_map_handler(comms_map_handler_t handler);

/**
 * Copies the last barcode map the handler accepted into p_out (3 bytes).
 * Returns false if none has been accepted since boot. Also sent in the
 * heartbeat, so the laptop can see which map the car holds.
 */
bool         comms_get_map(uint8_t p_out[COMMS_MAP_LEN]);

/** Count a telemetry event lost to a full queue (reported in the heartbeat). */
void         comms_note_event_dropped(void);

/**
 * Publish len bytes to <MQTT_TOPIC_ROOT>/<p_subtopic>.
 * @param qos  0 = fire-and-forget (live data), 1 = acknowledged (events)
 * @return false if offline or the output buffer is full (message dropped).
 */
bool         comms_publish(char const * p_subtopic, void const * p_data,
                           uint16_t len, uint8_t qos, bool b_retain);

#endif /* COMMS_H */

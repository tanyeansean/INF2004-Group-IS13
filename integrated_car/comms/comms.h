/**
 * @file    comms.h
 * @brief   Buddy 1 - WiFi + MQTT transport layer (Communication API, part 1).
 *
 * Owns: WiFi join, broker connection, automatic recovery, heartbeat,
 * and receiving commands and the barcode map from the laptop.
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

/** Connection state (also published in the heartbeat). */
typedef enum
{
    COMMS_WIFI_DOWN = 0,     /**< not associated, waiting to retry        */
    COMMS_WIFI_JOINING,      /**< association + DHCP in progress          */
    COMMS_MQTT_CONNECTING,   /**< WiFi up, connecting to the broker       */
    COMMS_ONLINE             /**< WiFi + MQTT up, telemetry flowing       */
} comms_link_t;

/** Commands the laptop can send on <root>/cmd. */
typedef enum
{
    CMD_START = 0,           /**< "start"                                 */
    CMD_STOP,                /**< "stop"      - controlled stop           */
    CMD_ESTOP,               /**< "estop"     - emergency stop            */
    CMD_SET_SPEED,           /**< "speed N"   - arg = target speed, cm/s  */
    CMD_RESET_PEAK,          /**< "reset_peak" - clear stored hump peak   */
    CMD_UNKNOWN
} comms_cmd_type_t;

typedef struct
{
    comms_cmd_type_t type;
    int32_t          arg;    /**< only used by CMD_SET_SPEED              */
} comms_cmd_t;

/** Called from comms_poll() (main-loop context) for every valid command. */
typedef void (*comms_cmd_handler_t)(comms_cmd_t const * p_cmd);

/**
 * Called from comms_poll() (main-loop / Telemetry task context, never from
 * the lwIP callback) for every message on <root>/cmd/barcode_map.
 * p_payload (4-byte aligned) holds the bytes as received; len may be 0, or
 * anything other than 3 if the laptop sent the wrong thing (at most 48 bytes
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

/** Link counters, included in the heartbeat. */
typedef struct
{
    uint32_t reconnects;     /**< times the link dropped after being online */
    uint32_t pub_ok;
    uint32_t pub_fail;       /**< publish refused (buffer full / offline)   */
    uint32_t cmds_rx;        /**< text commands received                    */
    uint32_t maps_rx;        /**< barcode-map messages received             */
    uint32_t cmds_dropped;   /**< receive queue full or command too long    */
} comms_stats_t;

/** Initialise the WiFi chip and MQTT client. Returns false if the chip fails. */
bool         comms_init(void);

/** Run the connection state machine, heartbeat and command dispatch. */
void         comms_poll(void);

comms_link_t comms_get_link(void);
bool         comms_is_online(void);
char const * comms_link_name(comms_link_t link);
void         comms_get_stats(comms_stats_t * p_stats);

/** Register the Vehicle Controller's command handler (one handler). */
void         comms_set_cmd_handler(comms_cmd_handler_t handler);

/** Register the barcode-map handler (one handler; telem_init() does this). */
void         comms_set_map_handler(comms_map_handler_t handler);

/**
 * Copies the last barcode map the handler accepted into p_out (3 bytes).
 * Returns false if none has been accepted since boot. Also sent in the
 * heartbeat as "map", so the laptop can see which map the car holds.
 */
bool         comms_get_map(uint8_t p_out[COMMS_MAP_LEN]);

/**
 * Publish a text payload to <MQTT_TOPIC_ROOT>/<p_subtopic>.
 * @param qos  0 = fire-and-forget (live data), 1 = acknowledged (events)
 * @return false if offline or the output buffer is full (message dropped).
 */
bool         comms_publish(char const * p_subtopic, char const * p_payload,
                           uint8_t qos, bool b_retain);

/** Same as comms_publish() for a binary payload of len bytes (may contain 0x00). */
bool         comms_publish_bin(char const * p_subtopic, void const * p_data,
                               uint16_t len, uint8_t qos, bool b_retain);

#endif /* COMMS_H */

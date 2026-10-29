/**
 * @file    comms_config.h
 * @brief   Everything you need to EDIT before building: WiFi, broker, rates.
 *
 * Do not commit your real WiFi password to the team repo: put it in
 * wifi_secrets.h (gitignored), copied from wifi_secrets.example.h.
 */
#ifndef COMMS_CONFIG_H
#define COMMS_CONFIG_H

/* ---- WiFi (2.4 GHz WPA2-Personal only - a phone hotspot works best) ----- */
/* ---- MQTT broker IP (the laptop running mosquitto) ---------------------- */
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#warning "wifi_secrets.h (next to comms_config.h) not found - using placeholders from wifi_secrets.example.h"
#include "wifi_secrets.example.h"
#endif

#define MQTT_BROKER_PORT        (1883u)
#define MQTT_KEEPALIVE_S        (10u)              /* broker declares us dead after ~15 s */

/* All topics start with this. Change to your team code.                     */
#define MQTT_TOPIC_ROOT         "robocar/p2-4"

/* ---- Timing ------------------------------------------------------------- */
#define TELEM_PERIOD_MS         (200u)    /* live telemetry at 5 Hz            */
#define ODOM_PERIOD_MS          (1000u)   /* encoder counts at 1 Hz            */
#define HEARTBEAT_PERIOD_MS     (1000u)   /* heartbeat at 1 Hz                 */
#define WIFI_JOIN_TIMEOUT_MS    (15000u)  /* give up on one WiFi attempt       */
#define MQTT_CONNECT_TIMEOUT_MS (8000u)   /* give up on one broker attempt     */
#define RECONNECT_MIN_MS        (1000u)   /* back-off starts here...           */
#define RECONNECT_MAX_MS        (16000u)  /* ...doubles up to here            */

/* ---- Limits ------------------------------------------------------------- */
#define CMD_SPEED_MAX_CMS       (20u)     /* speed commands capped at 0.2 m/s  */

#endif /* COMMS_CONFIG_H */

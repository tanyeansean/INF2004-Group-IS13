/**
 * @file    comms_config.h
 * @brief   Buddy 1 - topics and rates (uT-Kernel port of integrated_car's).
 *
 * WiFi name, password and broker IP are NOT here: the kernel's WiFi task
 * reads them from config/wifi_secrets.h (gitignored, copy
 * config/wifi_secrets.example.h). Build with: make NET=lwip MQTT=1
 */
#ifndef COMMS_CONFIG_H
#define COMMS_CONFIG_H

/* All topics start with this. Same as integrated_car, so the laptop's
 * tools/telemetry_monitor.py works with either firmware. */
#define MQTT_TOPIC_ROOT         "robocar/p2-4"

/* ---- Timing ------------------------------------------------------------- */
#define TELEM_PERIOD_MS         (200u)    /* live telemetry at 5 Hz            */
#define ODOM_PERIOD_MS          (1000u)   /* encoder counts at 1 Hz            */
#define HEARTBEAT_PERIOD_MS     (1000u)   /* heartbeat at 1 Hz                 */

/* ---- Limits ------------------------------------------------------------- */
#define CMD_SPEED_MAX_CMS       (20u)     /* speed commands capped at 0.2 m/s  */

#endif /* COMMS_CONFIG_H */

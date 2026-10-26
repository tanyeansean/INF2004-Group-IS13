/* Copy this file to wifi_secrets.h (same folder) and fill in your own values.
 * wifi_secrets.h is gitignored, so your real password is never committed.
 * Without it the build uses the placeholders below and WiFi will not join. */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID       "YOUR_HOTSPOT_NAME"
#define WIFI_PASSWORD   "YOUR_HOTSPOT_PASSWORD"
#define MQTT_BROKER_IP  "192.168.43.100"   /* laptop IP on the hotspot (ipconfig) */

#endif /* WIFI_SECRETS_H */

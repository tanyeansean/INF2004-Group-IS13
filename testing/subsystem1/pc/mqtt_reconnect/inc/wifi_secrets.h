/* PC test only: dummy values, no real network. mqtt_utk.c includes
 * "wifi_secrets.h", so the test supplies this stand-in. */
#define MQTT_BROKER_IP   "10.0.0.2"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "test/heartbeat"

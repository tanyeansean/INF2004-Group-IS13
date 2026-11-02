#!/bin/sh
# PC test of the reconnect logic in lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c,
# with fake lwIP/MQTT/timer headers (needs gcc). From the repo root:
#   sh path/to/mqtt_reconnect/run.sh path/to/mqtt_reconnect
# An unreachable broker is simulated twice: connect attempts failing after
# 1 s, and after 20 s (a host that never answers).
set -e
T=$(cd "$1" && pwd)
gcc -std=gnu11 -Wall -Wextra -I "$T/inc" -I lib/liblwip/sysdepend/pico_rp2040 \
    -I lib/libwifi/sysdepend/pico_rp2040 \
    "$T/harness.c" lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c -o /tmp/mqtt_reconnect_test
/tmp/mqtt_reconnect_test 1000
/tmp/mqtt_reconnect_test 20000

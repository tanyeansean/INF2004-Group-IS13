#!/bin/sh
# Subsystem 1 PC tests (no Pico needed; needs gcc). Run from the repo root:
#   sh testing/subsystem1/pc/run_pc_tests.sh
# 1. Safety Monitor attendance logic (safety_logic.c) on a simulated 10 ms tick.
# 2. MQTT reconnect pacing in lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c.
set -e
D=testing/subsystem1/pc
echo "== Safety Monitor logic"
gcc -std=c11 -Wall -Wextra -Werror -I app_program/subsystem1 \
    $D/sm_test.c app_program/subsystem1/safety_logic.c -o /tmp/s1_sm_test
/tmp/s1_sm_test
echo "== MQTT reconnect"
sh $D/mqtt_reconnect/run.sh $D/mqtt_reconnect

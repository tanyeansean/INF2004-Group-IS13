#!/bin/sh
# Laptop test for subsystem 4's detector.  Run from anywhere:
#   sh testing/subsystem4/pc/run.sh
# Needs gcc.  Builds terrain_logic.c exactly as the car uses it.
set -e
cd "$(dirname "$0")"
gcc -std=c99 -Wall -Wextra -O1 -o terrain_sim terrain_sim.c \
    ../../../app_program/subsystem4/terrain_logic.c -lm
./terrain_sim

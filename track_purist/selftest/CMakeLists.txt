# Buddy 3 Track Purist bare-metal self-test (no uT-Kernel).
# Modeled on the working ultrasonic lab so it builds with the
# VS Code Raspberry Pi Pico extension on this machine.

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(track_purist_selftest C CXX ASM)

pico_sdk_init()

add_executable(track_purist_selftest
        track_purist_selftest.c
        ../src/line_follow.c
        ../src/ir_calibrate.c
        ../src/barcode_decode.c
        ../src/nav_cmd.c
)

pico_set_program_name(track_purist_selftest "track_purist_selftest")
pico_set_program_version(track_purist_selftest "0.1")

# Output over USB serial (so you can read it with a USB cable).
pico_enable_stdio_uart(track_purist_selftest 0)
pico_enable_stdio_usb(track_purist_selftest 1)

target_link_libraries(track_purist_selftest pico_stdlib hardware_adc)

target_include_directories(track_purist_selftest PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../include
)

pico_add_extra_outputs(track_purist_selftest)

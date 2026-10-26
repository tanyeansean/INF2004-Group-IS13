# Integrated car firmware (bare metal, no uT-Kernel):
#   Buddy 3 track_purist (from the team repo) + Buddy 1 robocar_comms v0.2
#   (copied into comms/) + motor.c.

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

project(integrated_car C CXX ASM)

pico_sdk_init()

# Buddy 3's code is used straight from track_purist/, never copied.
# Inside the repo it is ../track_purist; outside (old test location) it is
# ../INF2004-Group-IS13/track_purist. Override with -DTRACK_PURIST_DIR=...
if (NOT TRACK_PURIST_DIR)
    if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/../track_purist/src/nav_cmd.c)
        set(TRACK_PURIST_DIR ${CMAKE_CURRENT_LIST_DIR}/../track_purist)
    else()
        set(TRACK_PURIST_DIR ${CMAKE_CURRENT_LIST_DIR}/../INF2004-Group-IS13/track_purist)
    endif()
endif()
if (NOT EXISTS ${TRACK_PURIST_DIR}/src/nav_cmd.c)
    message(FATAL_ERROR "track_purist not found at ${TRACK_PURIST_DIR}. "
        "Check out Buddy 3's track_purist/ next to integrated_car/, "
        "or pass -DTRACK_PURIST_DIR=<path>.")
endif()
message(STATUS "Using track_purist from ${TRACK_PURIST_DIR}")

set(OWN_SOURCES
    main.c
    motor.c
    comms/comms.c
    comms/telemetry.c
)
set(TP_SOURCES
    ${TRACK_PURIST_DIR}/src/line_follow.c
    ${TRACK_PURIST_DIR}/src/ir_calibrate.c
    ${TRACK_PURIST_DIR}/src/barcode_decode.c
    ${TRACK_PURIST_DIR}/src/nav_cmd.c
)

add_executable(integrated_car ${OWN_SOURCES} ${TP_SOURCES})

target_include_directories(integrated_car PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/comms        # lwipopts.h, comms_config.h
    ${TRACK_PURIST_DIR}/include
)

set_source_files_properties(${OWN_SOURCES} ${TP_SOURCES}
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

target_link_libraries(integrated_car
    pico_stdlib
    pico_unique_id
    hardware_adc
    hardware_pwm
    hardware_clocks
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

pico_set_program_name(integrated_car "integrated_car")
pico_set_program_version(integrated_car "0.1")

pico_enable_stdio_usb(integrated_car 1)
pico_enable_stdio_uart(integrated_car 0)

pico_add_extra_outputs(integrated_car)

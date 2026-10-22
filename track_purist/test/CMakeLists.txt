# Host unit-test build for Buddy 3 (Track Purist) decode/line logic.
# Builds native executables on the dev machine (no Pico SDK / RTOS needed).
#
#   cmake -S . -B build
#   cmake --build build
#   ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.13)
project(track_purist_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT MSVC)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TP_INC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(TP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src)

enable_testing()

# Barcode decode tests: compile the core with BARCODE_HOST_TEST to exclude
# the ISR/task (RTOS-dependent) section.
add_executable(test_barcode
    test_barcode.c
    ${TP_SRC}/barcode_decode.c)
target_include_directories(test_barcode PRIVATE ${TP_INC} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(test_barcode PRIVATE BARCODE_HOST_TEST)
add_test(NAME barcode COMMAND test_barcode)

# Line-follow + calibration tests.
add_executable(test_line_follow
    test_line_follow.c
    ${TP_SRC}/line_follow.c
    ${TP_SRC}/ir_calibrate.c)
target_include_directories(test_line_follow PRIVATE ${TP_INC} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(test_line_follow PRIVATE LINE_FOLLOW_HOST_TEST)
add_test(NAME line_follow COMMAND test_line_follow)

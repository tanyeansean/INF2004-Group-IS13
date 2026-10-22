# Buddy 3 (Track Purist) module — target build fragment.
#
# Add to the top-level Pico project with:
#   add_subdirectory(track_purist)
#   target_link_libraries(<app> track_purist)
#
# Provides the line-follow + barcode-decode logic, the IR calibration
# helper, and the uT-Kernel RTOS adapter. The integration layer must
# supply the extern globals (queue/flag ids, board ADC/time hooks)
# declared in line_follow.c and barcode_decode.c.
#
# NOTE: rtos_port_utk.c is currently a STUB (project risk A7). Complete
# its TODOs once uT-Kernel headers are on the include path.

add_library(track_purist STATIC
    src/line_follow.c
    src/barcode_decode.c
    src/ir_calibrate.c
    src/rtos_port_utk.c)

target_include_directories(track_purist PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

# When uT-Kernel is available, link it here, e.g.:
# target_link_libraries(track_purist PUBLIC utkernel)

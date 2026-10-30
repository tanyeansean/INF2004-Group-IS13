################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# Buddy 3's pure logic (line follow, calibration, Code 39 decoder, nav_cmd),
# shared with the host unit tests and the bare-metal integrated_car bench.
# The RTOS task bodies inside line_follow.c / barcode_decode.c are left out
# (LINE_FOLLOW_HOST_TEST / BARCODE_HOST_TEST): the uT-Kernel tasks live in
# app_program/subsystem3/.  rtos_port_utk.c is therefore not built.
TP_SRCS = line_follow.c ir_calibrate.c barcode_decode.c nav_cmd.c

TP_OBJS = $(addprefix ./mtkernel_3/track_purist/src/,$(TP_SRCS:.c=.o))

OBJS += $(TP_OBJS)
C_DEPS += $(TP_OBJS:.o=.d)

mtkernel_3/track_purist/src/%.o: ../track_purist/src/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -DLINE_FOLLOW_HOST_TEST -DBARCODE_HOST_TEST -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

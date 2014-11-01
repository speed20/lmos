include config.mk
include $(ROOT_DIR)/kernel/build.mk
include $(ROOT_DIR)/module/build.mk
include $(ROOT_DIR)/hal/build.mk
include $(ROOT_DIR)/lib/build.mk

header	=	-I include \
			-I ${ROOT_DIR}/lib \
			-I ${ROOT_DIR}/lib/CMSIS/Include \
			-I ${ROOT_DIR}/lib/CMSIS/Device/ST/STM32F4xx/Include \
			-I lib/STemWinLibrary522/inc/
#			-I ~/Work/arm-none-eabi/include/


boot_code_path = boot
VPATH += $(boot_code_path):app:lib/bsp:task:../../lib/CMSIS/DSP_Lib/Source/FastMathFunctions
asm_objs = $(addprefix $(OUT_DIR)/,$(addsuffix .o,$(basename $(notdir $(wildcard $(boot_code_path)/*.S)))))
objs += $(addprefix $(OUT_DIR)/,$(addsuffix .o,$(basename $(notdir $(wildcard $(boot_code_path)/*.c)))))

tmp_objs  = main.o \
		stm32f429i_discovery.o \
		stm32f429i_discovery_sdram.o \
		stm32f429i_discovery_lcd.o \
		isr.o \
		GUIConf.o \
		LCDConf.o \
		GUI_X.o \
		task_flash.o
#		task_pmon.o \
		task_pulse.o

#		task_dac.o \
#		task_powerbox.o \
#		task_mpu6050.o \
#		usb_bsp.o \
#		task_asr.o \
#		task_mpu6050.o \
		arm_sin_f32.o \
		task_ir.o

objs += $(addprefix $(OUT_DIR)/,$(tmp_objs))

CFLAGS += \
		 $(header) \
         -D USE_DEFAULT_TIMEOUT_CALLBACK \
         -D USE_USB_OTG_FS \
         -D USE_STDPERIPH_DRIVER \
         -D ARM_MATH_CM4 \
		 -D STM32F429_439xx \
		 -D USE_STM32F429I_DISCO \
		 -D ARM_MATH_CM4 \
		 -D ARM_MATH_MATRIX_CHECK \
		 -D ARM_MATH_ROUNDING \
		 -D __FPU_PRESENT \
		 -D __FPU_USED

ld_script	= stm32f40x.ld
image		= $(OUT_DIR)/Demo.elf

.PHONY: demo clean show

demo: $(image)

#'../../lib/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a'
$(image): $(objs) $(asm_objs)
	@echo "LD $(image)"
	@${LD} -T ${ld_script} \
           --entry main \
           ${LDFLAGS} -o $@ $^  \
           '${LIBC}' '${LIBM}' '${LIBGCC}' 'lib/STemWinLibrary522/Lib/STemWin522_CM4_OS_GCC.a'
	${OBJCOPY} -O binary $@ ${@:.elf=.bin}

#	${STRIP} $@

$(OUT_DIR):
ifeq ($(filter $(OUT_DIR),$(wildcard *)), )
	$(shell mkdir $(OUT_DIR))
endif

test:
	@echo $(VPATH)
	@echo $(objs)

$(objs): $(OUT_DIR)/%.o : %.c
	@echo "CC $(notdir $@)"
	@$(CC) $(CFLAGS) -o $@ -c $<

$(asm_objs): $(OUT_DIR)/%.o : %.S
	@echo "AS $(notdir $@)"
	@$(CC) $(AFLAGS) -o $@ -c $<

clean:
	-rm out -rf

show:
	@echo $(objs)

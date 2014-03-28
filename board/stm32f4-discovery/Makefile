include config.mk
include $(ROOT_DIR)/kernel/build.mk
include $(ROOT_DIR)/module/build.mk
include $(ROOT_DIR)/hal/build.mk
include $(ROOT_DIR)/lib/build.mk

header	=	-I include \
			-I ${ROOT_DIR}/lib \
			-I ${ROOT_DIR}/lib/CMSIS/Include \
			-I ${ROOT_DIR}/lib/CMSIS/Device/ST/STM32F4xx/Include

boot_code_path = boot
VPATH += $(boot_code_path):app:interrupt:bsp:task
asm_objs = $(addprefix $(OUT_DIR)/,$(addsuffix .o,$(basename $(notdir $(wildcard $(boot_code_path)/*.S)))))
objs += $(addprefix $(OUT_DIR)/,$(addsuffix .o,$(basename $(notdir $(wildcard $(boot_code_path)/*.c)))))

tmp_objs  = main.o \
		stm32f429i_discovery.o \
		isr.o \
		task_asr.o \
		task_flash.o \
		task_mpu6050.o \
		task_pulse.o \
		task_ir.o

objs += $(addprefix $(OUT_DIR)/,$(tmp_objs))

CFLAGS += \
		 $(header) \
         -D USE_DEFAULT_TIMEOUT_CALLBACK \
         -D USE_USB_OTG_FS \
         -D USE_STDPERIPH_DRIVER \
         -D ARM_MATH_CM4 \
		 -D STM32F429_439xx
         #-D MOD_MTHOMAS_STMLIB \
         #-D GCC_ARMCM4F_LM3S102 \

ld_script	= stm32f40x.ld
image		= $(OUT_DIR)/Demo.elf

.PHONY: demo clean show

demo: $(image)

$(image): $(objs) $(asm_objs)
	@echo "LD $(image)"
	@${LD} -T ${ld_script} \
           --entry main \
           ${LDFLAGS} -o $@ $^  \
           '${LIBC}' '${LIBM}' '${LIBGCC}'
	${OBJCOPY} -O binary $@ ${@:.elf=.bin}

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

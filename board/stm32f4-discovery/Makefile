include config.mk

lib_inc := $(patsubst %,-I %,$(shell find ${SOURCE_ROOT}/lib -type d -print))
#$(info ${lib_inc})

header	=	-I include \
			-I ${SOURCE_ROOT}/hal/include \
			-I ${SOURCE_ROOT}/module \
			-I ${SOURCE_ROOT}/kernel/Source \
			-I ${SOURCE_ROOT}/kernel/Source/include \
			-I ${SOURCE_ROOT}/kernel/Source/portable/GCC/ARM_CM4F \
			$(lib_inc)

src_path= $(shell find . -type d -print)

boot_code_path = boot
boot_objs = $(addprefix $(OUT_DIR)/,$(addsuffix .o,$(basename $(notdir $(wildcard $(boot_code_path)/*.S)))))

tmp_objs  = main.o \
		isr.o \
		stm32f4_discovery.o \
		task_asr.o \
		task_flash.o \
		task_mpu6050.o \
		task_pulse.o \
		task_ir.o

objs = $(addprefix $(OUT_DIR)/,$(tmp_objs))

VPATH = $(src_path)

CFLAGS += $(patsubst %,-I %,$(src_path))
CFLAGS += \
		 $(header) \
         -I data \
         -D GCC_ARMCM4F_LM3S102 \
         -D USE_STDPERIPH_DRIVER \
         -D USE_DEFAULT_TIMEOUT_CALLBACK \
         -D USE_USB_OTG_FS \
         -D STM32F4XX \
         -D USE_STDPERIPH_DRIVER \
         -D MOD_MTHOMAS_STMLIB \
         -D ARM_MATH_CM4 \
         -D DEBUG

ld_script	= stm32f40x.ld
image		= $(OUT_DIR)/Demo.elf

.PHONY: demo clean show

demo: $(image)

$(image): $(objs) $(boot_objs) other
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
	@echo $(objs)

$(objs): $(OUT_DIR)/%.o : %.c
	@echo "CC $@"
	@$(CC) $(CFLAGS) -o $@ -c $<

$(boot_objs): $(OUT_DIR)/%.o : %.S
	@echo "AS $@"
	@$(CC) $(AFLAGS) -o $@ -c $<

other:
	make -C ${SOURCE_ROOT}

clean:
	-rm out/*

show:
	@echo $(objs)

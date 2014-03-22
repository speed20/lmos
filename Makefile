CROSS_COMPILE = arm-none-eabi-
OUT_DIR := out

CC	= $(CROSS_COMPILE)cc
AR	= $(CROSS_COMPILE)ar
LD	= $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

AFLAGS	= -mthumb -mcpu=cortex-m4 -O3
CFLAGS	= -mthumb -mcpu=cortex-m4 -O3
LDFLAGS	= -Map $(OUT_DIR)/out.map

LIBGCC	= $(shell $(CC) -mthumb -march=armv6t2 -print-libgcc-file-name)
LIBC	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libc.a)
LIBM	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libm.a)

lib_inc := $(patsubst %,-I %,$(shell find lib -type d -print))

header	=	-I include \
			-I kernel/Source \
			-I kernel/Source/include \
			-I kernel/Source/portable/GCC/ARM_CM4F \
			$(lib_inc)

src_path=	. \
			boot \
			task \
			kernel/Source \
			kernel/Source/portable/GCC/ARM_CM4F \
			kernel/Source/include/portable/GCC/ARM_CM4F \
			kernel/Source/portable/MemMang \
			lib/eMPL \
			lib/STM32F4xx_StdPeriph_Driver/src \
			lib/STM32F4-Discovery \
			lib/STM32_USB_Device_Library/Core/src \
			lib/STM32_USB_Device_Library/Class/hid/src \
			lib/STM32_USB_Device_Library/Class/cdc/src \
			lib/STM32_USB_OTG_Driver/src \
			driver/interrupt \
			driver/bus/usb \
			driver/bus/spi \
			driver/bus/spi/chip \
			driver/bus/serial \
			driver/bus/i2c \
			driver/bus/i2c/chip \
			driver/device/timer \
			driver/device/voice \
			driver/device/wireless \
			driver/device/hmc588xx \
			driver/device/misc \
			driver/device/led \
			driver/device/display \
			driver/device/invense \
			driver/device/invense/mpulib

boot_code_path = boot
boot_objs = $(addprefix $(OUT_DIR)/,$(addsuffix .o,$(basename $(notdir $(wildcard $(boot_code_path)/*.S)))))

tmp_objs  = main.o \
		list.o    \
		queue.o   \
		tasks.o   \
		port.o    \
		timers.o    \
		heap_2.o  \
		croutine.o \
		misc.o 	\
		system_stm32f4xx.o \
		stm32f4xx_exti.o \
		stm32f4xx_gpio.o \
		stm32f4xx_rcc.o \
		stm32f4xx_usart.o \
		stm32f4xx_syscfg.o \
		stm32f4xx_adc.o \
		stm32f4xx_spi.o \
		stm32f4xx_i2c.o \
		stm32f4xx_tim.o \
		stm32f4xx_it.o \
		stm32f4xx_flash.o \
		stm32f4xx_dma.o \
		stm32f4_discovery.o \
		usbd_core.o \
		usbd_ioreq.o \
		usbd_req.o \
		usbd_hid_core.o \
		usbd_cdc_core.o \
		usbd_cdc_vcp.o \
		usbd_desc.o \
		usbd_usr.o \
		usb_dcd.o \
		usb_dcd_int.o \
		usb_core.o \
		usb_bsp.o \
		serial.o \
		i2c_bsp.o \
		mpulib.o \
		mpu6050.o \
		quaternion.o \
		inv_mpu_dmp_motion_driver.o \
		inv_mpu.o \
		vector3d.o \
		hmc5883l.o \
		delay.o \
		bt.o \
		ld3320.o \
		led.o \
		oled.o \
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

ld_script	= build/stm32f40x.ld
image		= $(OUT_DIR)/Demo.elf

.PHONY: demo clean show

demo: $(image)

$(image): $(objs) $(boot_objs)
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

clean:
	-rm out/*

show:
	@echo $(objs)

export ROOT_DIR = $(shell pwd)/../..
export CROSS_COMPILE = arm-none-eabi-
export OUT_DIR = $(shell pwd)/out

export CC	= $(CROSS_COMPILE)cc
export AR	= $(CROSS_COMPILE)ar
LD	= $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

export AFLAGS	:= -mthumb -mcpu=cortex-m4 -O2
export CFLAGS	:= -mthumb -mcpu=cortex-m4 -O2 -DSTM32F40XX
LDFLAGS	= -Map $(OUT_DIR)/out.map

LIBGCC	= $(shell $(CC) -mthumb -march=armv6t2 -print-libgcc-file-name)
LIBC	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libc.a)
LIBM	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libm.a)

ifeq ($(wildcard $(OUT_DIR)),)
$(shell mkdir $(OUT_DIR))
endif
#$(if $(filter $(wildcard $(OUT_DIR)),out),$(shell mkdir $(OUT_DIR)))

CONFIG_STM32_USB_HOST_Library=n

CONFIG_STM32F4xx_StdPeriph_Driver=y
CONFIG_stm32f4xx_misc=y
CONFIG_stm32f4xx_fsmc=n
CONFIG_stm32f4xx_dma2d=n
CONFIG_stm32f4xx_syscfg=y
CONFIG_stm32f4xx_gpio=y
CONFIG_stm32f4xx_hash_md5=n
CONFIG_stm32f4xx_rng=n
CONFIG_stm32f4xx_cryp_aes=n
CONFIG_stm32f4xx_adc=y
CONFIG_stm32f4xx_cryp_tdes=n
CONFIG_stm32f4xx_iwdg=n
CONFIG_stm32f4xx_dac=n
CONFIG_stm32f4xx_dcmi=n
CONFIG_stm32f4xx_cryp=n
CONFIG_stm32f4xx_ltdc=n
CONFIG_stm32f4xx_sai=n
CONFIG_stm32f4xx_wwdg=n
CONFIG_stm32f4xx_sdio=n
CONFIG_stm32f4xx_can=n
CONFIG_stm32f4xx_tim=y
CONFIG_stm32f4xx_dbgmcu=n
CONFIG_stm32f4xx_usart=y
CONFIG_stm32f4xx_spi=y
CONFIG_stm32f4xx_crc=n
CONFIG_stm32f4xx_pwr=y
CONFIG_stm32f4xx_hash=n
CONFIG_stm32f4xx_fmc=n
CONFIG_stm32f4xx_flash=y
CONFIG_stm32f4xx_exti=y
CONFIG_stm32f4xx_i2c=y
CONFIG_stm32f4xx_rcc=y
CONFIG_stm32f4xx_dma=y
CONFIG_stm32f4xx_hash_sha1=n
CONFIG_stm32f4xx_rtc=n
CONFIG_stm32f4xx_cryp_des=n

CONFIG_STM32_USB_Device_Library=y
CONFIG_Core=y
CONFIG_Class_audio=n
CONFIG_Class_cdc=y
CONFIG_Class_msc=n
CONFIG_Class_hid=y
CONFIG_Class_dfu=n

CONFIG_STM32_USB_OTG_Driver=y

define all_c_files
$(notdir $(wildcard $1/*.c))
endef

define all_header_dir
$(addprefix -I./,$(shell find $(1) -name inc))
endef

ifeq ($(CONFIG_STM32_USB_HOST_Library), y)
objs += $(patsubst %.c,%.o,$(call all_c_files,"bsp/STM32_USB_HOST_Library"))
CFLAGS += $(call all_header_dir,"bsp/STM32_USB_HOST_Library")
endif

ifeq ($(CONFIG_STM32F4xx_StdPeriph_Driver), y)
opt = $(basename $(call all_c_files,bsp/STM32F4xx_StdPeriph_Driver/src))
objs += $(foreach i,$(opt),$(if $(filter y,$(CONFIG_$(i))),$(OUT_DIR)/$(i).o))
CFLAGS += $(call all_header_dir,"bsp/STM32F4xx_StdPeriph_Driver")
VPATH += bsp/STM32F4xx_StdPeriph_Driver/src/
endif

ifeq ($(CONFIG_STM32_USB_Device_Library), y)
tmp_path := bsp/STM32_USB_Device_Library
objs += $(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(call all_c_files,$(tmp_path)/Core/src)))
CFLAGS += $(call all_header_dir,$(tmp_path)/Core)
VPATH += $(tmp_path)/Core/src

opt1=$(basename $(notdir $(shell ls $(tmp_path)/Class)))
objs += $(foreach i,$(opt1),$(if $(filter y,$(CONFIG_Class_$(i))),\
		$(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(call all_c_files,$(tmp_path)/Class/$i/src)))))

VPATH += $(foreach i,$(opt1),$(if $(filter y,$(CONFIG_Class_$(i))),$(tmp_path)/Class/$i/src))
CFLAGS += $(foreach i,$(opt1),$(if $(filter y,$(CONFIG_Class_$(i))),$(call all_header_dir,$(tmp_path)/Class/$i)))
endif

ifeq ($(CONFIG_STM32_USB_OTG_Driver), y)
objs += $(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(call all_c_files,bsp/STM32_USB_OTG_Driver/src)))
CFLAGS += $(call all_header_dir,"bsp/STM32_USB_OTG_Driver")
VPATH += bsp/STM32_USB_OTG_Driver/src
endif

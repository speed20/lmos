ifneq ($(OUT_DIR), )

hal_local_path := $(ROOT_DIR)/hal

define all_c_files
$(notdir $(wildcard $1/*.c))
endef

define all_header_dir
$(addprefix -I./,$(shell find $(1) -name inc))
endef

CONFIG_HAL_i2c=n
CONFIG_HAL_serial=y
CONFIG_HAL_spi=n
CONFIG_HAL_time=y
CONFIG_HAL_usb=y

ifeq ($(CONFIG_HAL_i2c), y)
VPATH += $(hal_local_path)/i2c
hal_src += $(call all_c_files,$(hal_local_path)/i2c)
endif

ifeq ($(CONFIG_HAL_serial), y)
VPATH += $(hal_local_path)/serial
hal_src += $(call all_c_files,$(hal_local_path)/serial)
endif

ifeq ($(CONFIG_HAL_spi), y)
VPATH += $(hal_local_path)/spi
hal_src += $(call all_c_files,$(hal_local_path)/spi)
endif

ifeq ($(CONFIG_HAL_time), y)
VPATH += $(hal_local_path)/time
hal_src += $(call all_c_files,$(hal_local_path)/time)
endif

ifeq ($(CONFIG_HAL_usb), y)
VPATH += $(hal_local_path)/usb
ifeq ($(CONFIG_Class_cdc), y)
hal_src += usbd_cdc_vcp.c
endif
hal_src += usbd_desc.c
hal_src += usbd_usr.c
endif

hal_objs := $(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(hal_src)))
objs += $(hal_objs)
CFLAGS += -I$(ROOT_DIR)/hal/include
endif


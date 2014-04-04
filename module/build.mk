ifneq ($(OUT_DIR), )
module_local_path := $(ROOT_DIR)/module

define all_c_files
$(notdir $(wildcard $1/*.c))
endef

define all_header_dir
$(addprefix -I./,$(shell find $(1) -name inc))
endef

CONFIG_asr=n
CONFIG_timer=y
CONFIG_wireless=n
CONFIG_hmc588xx=y
CONFIG_misc=n
CONFIG_led=y
CONFIG_display=n
CONFIG_invense=y

ifeq ($(CONFIG_asr), y)
VPATH += $(module_local_path)/asr
module_src += $(call all_c_files,$(module_local_path)/asr)
endif

ifeq ($(CONFIG_timer), y)
VPATH += $(module_local_path)/timer
module_src += $(call all_c_files,$(module_local_path)/timer)
endif

ifeq ($(CONFIG_wireless), y)
VPATH += $(module_local_path)/wireless
module_src += $(call all_c_files,$(module_local_path)/wireless)
endif

ifeq ($(CONFIG_hmc588xx), y)
VPATH += $(module_local_path)/hmc588xx
module_src += $(call all_c_files,$(module_local_path)/hmc588xx)
endif

ifeq ($(CONFIG_misc), y)
VPATH += $(module_local_path)/misc
module_src += $(call all_c_files,$(module_local_path)/misc)
endif

ifeq ($(CONFIG_led), y)
VPATH += $(module_local_path)/led
module_src += $(call all_c_files,$(module_local_path)/led)
endif

ifeq ($(CONFIG_display), y)
VPATH += $(module_local_path)/display
module_src += $(call all_c_files,$(module_local_path)/display)
endif

ifeq ($(CONFIG_invense), y)
VPATH += $(module_local_path)/invense
module_src += $(call all_c_files,$(module_local_path)/invense)
endif

objs += $(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(module_src)))

CFLAGS += -I$(module_local_path)
endif

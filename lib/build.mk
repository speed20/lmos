ifneq ($(OUT_DIR), )

lib_local_path := $(ROOT_DIR)/lib

define all_c_files
$(notdir $(wildcard $1/*.c))
endef

define all_header_dir
$(addprefix -I./,$(shell find $(1) -name inc))
endef

CONFIG_LIB_CMSIS=n
CONFIG_LIB_eMPL=y
CONFIG_LIB_mpulib=y

ifeq ($(CONFIG_LIB_CMSIS), y)
VPATH += $(lib_local_path)/CMSIS
CFLAGS += -I$(lib_local_path)/CMSIS
lib_src += $(call all_c_files,$(lib_local_path)/CMSIS)
endif

ifeq ($(CONFIG_LIB_eMPL), y)
VPATH += $(lib_local_path)/eMPL
CFLAGS += -I$(lib_local_path)/eMPL
lib_src += $(call all_c_files,$(lib_local_path)/eMPL)
endif

ifeq ($(CONFIG_LIB_mpulib), y)
VPATH += $(lib_local_path)/mpulib
CFLAGS += -I$(lib_local_path)/mpulib
lib_src += $(call all_c_files,$(lib_local_path)/mpulib)
endif

objs += $(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(lib_src)))
VPATH += $(dir $(lib_src))
endif

ifneq ($(OUT_DIR), )
SOC := ARM_CM4F
MEM_MANAGMENT := heap_2
kernel_dir := kernel/FreeRTOS/Source

kernel_src += $(wildcard $(ROOT_DIR)/$(kernel_dir)/*.c)
kernel_src += $(wildcard $(ROOT_DIR)/$(kernel_dir)/portable/GCC/$(SOC)/*.c)
kernel_src += $(ROOT_DIR)/$(kernel_dir)/portable/MemMang/$(MEM_MANAGMENT).c

VPATH += $(ROOT_DIR)/$(kernel_dir)
VPATH += $(ROOT_DIR)/$(kernel_dir)/portable/GCC/$(SOC)
VPATH += $(ROOT_DIR)/$(kernel_dir)/portable/MemMang

objs += $(addprefix $(OUT_DIR)/,$(patsubst %.c,%.o,$(notdir $(kernel_src))))
CFLAGS += $(addprefix -I,$(dir $(kernel_src)))
CFLAGS += -I$(ROOT_DIR)/$(kernel_dir)/include
endif


CROSS_COMPILE = arm-none-eabi-
OUT_DIR := out

CC	= $(CROSS_COMPILE)cc
AR	= $(CROSS_COMPILE)ar
LD	= $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

AFLAGS	= -mthumb -mcpu=cortex-m4 -O2
CFLAGS	= -mthumb -mcpu=cortex-m4 -O2
LDFLAGS	= -Map $(OUT_DIR)/out.map

LIBGCC	= $(shell $(CC) -mthumb -march=armv6t2 -print-libgcc-file-name)
LIBC	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libc.a)
LIBM	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libm.a)

src_path = $(shell find . -name kernel -prune -o -type d -print)
#objs1 := $(patsubst %.c,%.o,$(wildcard $(src_path)/*.c))
#objs2 := $(patsubst %.S,%.o,$(wildcard $(src_path)/*.S))

objs1 = $(notdir $(patsubst %c,%o,$(foreach n,$(src_path),$(wildcard $(n)/*.c))))
objs2 = $(notdir $(patsubst %S,%o,$(foreach n,$(src_path),$(wildcard $(n)/*.S))))

kernel_path = \
	kernel/Source \
	kernel/Source/portable/GCC/ARM_CM4F



kernel_objs = $(notdir $(patsubst %c,%o,$(foreach n,$(kernel_path),$(wildcard $(n)/*.c))))

VPATH = $(src_path) $(kernel_path)

#$(info $(objs1))
#$(info $(objs2))
#$(info $(kernel_objs))

CFLAGS += $(patsubst %,-I%,$(src_path))
CFLAGS += \
		 -I ./kernel/Source/include \
		 -I ./kernel/Source/portable/GCC/ARM_CM4F \
		 -I ./kernel/Source/include/portable/GCC/ARM_CM4F \
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

$(info $(CFLAGS))

$(kernel_objs): %o : %c
	$(CC) $(CFLAGS) -o $(OUT_DIR)/$@ -c $<

$(objs1): %o : %c
	$(CC) $(CFLAGS) -o $(OUT_DIR)/$@ -c $<

$(objs2): %o : %S
	$(CC) $(AFLAGS) -o $(OUT_DIR)/$@ -c $<

.PHONY: all
all: $(kernel_objs) $(objs1) $(objs2)
	$(CC) -o test $(objs1) $(objs2) $(kernel_objs)

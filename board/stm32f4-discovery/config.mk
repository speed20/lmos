SOURCE_ROOT = ../../
CROSS_COMPILE = arm-none-eabi-
OUT_DIR := out

CC	= $(CROSS_COMPILE)cc
AR	= $(CROSS_COMPILE)ar
LD	= $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

AFLAGS	= -mthumb -mcpu=cortex-m4 -O3
CFLAGS	= -mthumb -mcpu=cortex-m4 -O3 -DSTM32F40XX
LDFLAGS	= -Map $(OUT_DIR)/out.map

LIBGCC	= $(shell $(CC) -mthumb -march=armv6t2 -print-libgcc-file-name)
LIBC	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libc.a)
LIBM	= $(shell $(CC) -mthumb -march=armv6t2 -print-file-name=libm.a)



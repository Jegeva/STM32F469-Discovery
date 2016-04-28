CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src 


CFLAGS  = -g -Os -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -I../../CMSIS/Include -Iinc -DARM_MATH_CM4
CFLAGS += -DSTM32F469xx

SRC_DIR = src/
BUILD_DIR = build/
SRCS = $(notdir $(wildcard $(SRC_DIR)/*.c))


OBJS = $(SRCS:%.c=$(BUILD_DIR)%.o)

all: libstm32f469_discovery.a

$(BUILD_DIR)%.o : $(SRC_DIR)%.c
	$(CC) $(CFLAGS) -c -o $@ $^

libstm32f469_discovery.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f469_discovery.a

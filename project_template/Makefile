# put your *.o targets here, make should handle the rest!

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size
GDB=gdb-multiarch
STRIP=arm-none-eabi-strip

CFLAGS  = -g -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -DSTM32F469xx
CFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections
CFLAGS += -Os

FLASH_SIZE = 2097152
###################################################

vpath %.c src

ROOT=$(shell pwd)

CFLAGS += -Iinc -I../STM32F469-Discovery_lib/inc -I../../CMSIS/Include/
CFLAGS += -DARM_MATH_CM4 -lc -specs=nosys.specs

SRCS += ../STM32F469-Discovery_lib/startup_stm32f469xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

B = main
###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C ../STM32F469-Discovery_lib/

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -L../STM32F469-Discovery_lib -L../../CMSIS/Lib/GCC/ -lstm32f469_discovery -larm_cortexM4lf_math
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@perl -e 'printf "USED : %0.2f%% of '$(FLASH_SIZE)' bytes\n",(100*'`ls -l main.bin |cut -d' ' -f5`')/'$(FLASH_SIZE)


prog: 
	st-flash write $(PROJ_NAME).bin 0x8000000

gdb :
	openocd --file /usr/share/openocd/scripts/board/stm32f469discovery.cfg &
	sleep 1
	echo "target extended localhost:3333\nmonitor reset halt\nmonitor reset_nag enable\nb "$(B)"\nc\n" >.gdbinit
	gdb-multiarch main.elf && killall openocd
clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

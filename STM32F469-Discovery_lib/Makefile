CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src 


CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -I../../CMSIS/Include -Iinc -DARM_MATH_CM4
CFLAGS += -DSTM32F469xx


BUILD_DIR = build/

SRCS =cs43l22.c\
exc7200.c\
font12.c\
font16.c\
font20.c\
font24.c\
font8.c\
ft6x06.c\
ili9325.c\
ili9341.c\
l3gd20.c\
lis302dl.c\
lis3dsh.c\
lsm303dlhc.c\
mfxstm32l152.c\
otm8009a.c\
ov2640.c\
s5k5cag.c\
st7735.c\
stm32469i_discovery_audio.c\
stm32469i_discovery.c\
stm32469i_discovery_eeprom.c\
stm32469i_discovery_lcd.c\
stm32469i_discovery_qspi.c\
stm32469i_discovery_sd.c\
stm32469i_discovery_sdram.c\
stm32469i_discovery_ts.c\
stm32f4xx_hal_adc.c\
stm32f4xx_hal_adc_ex.c\
stm32f4xx_hal.c\
stm32f4xx_hal_can.c\
stm32f4xx_hal_cec.c\
stm32f4xx_hal_cortex.c\
stm32f4xx_hal_crc.c\
stm32f4xx_hal_cryp.c\
stm32f4xx_hal_cryp_ex.c\
stm32f4xx_hal_dac.c\
stm32f4xx_hal_dac_ex.c\
stm32f4xx_hal_dcmi.c\
stm32f4xx_hal_dcmi_ex.c\
stm32f4xx_hal_dma2d.c\
stm32f4xx_hal_dma.c\
stm32f4xx_hal_dma_ex.c\
stm32f4xx_hal_dsi.c\
stm32f4xx_hal_eth.c\
stm32f4xx_hal_flash.c\
stm32f4xx_hal_flash_ex.c\
stm32f4xx_hal_flash_ramfunc.c\
stm32f4xx_hal_fmpi2c.c\
stm32f4xx_hal_fmpi2c_ex.c\
stm32f4xx_hal_gpio.c\
stm32f4xx_hal_hash.c\
stm32f4xx_hal_hash_ex.c\
stm32f4xx_hal_hcd.c\
stm32f4xx_hal_i2c.c\
stm32f4xx_hal_i2c_ex.c\
stm32f4xx_hal_i2s.c\
stm32f4xx_hal_i2s_ex.c\
stm32f4xx_hal_irda.c\
stm32f4xx_hal_iwdg.c\
stm32f4xx_hal_lptim.c\
stm32f4xx_hal_ltdc.c\
stm32f4xx_hal_ltdc_ex.c\
stm32f4xx_hal_msp_template.c\
stm32f4xx_hal_nand.c\
stm32f4xx_hal_nor.c\
stm32f4xx_hal_pccard.c\
stm32f4xx_hal_pcd.c\
stm32f4xx_hal_pcd_ex.c\
stm32f4xx_hal_pwr.c\
stm32f4xx_hal_pwr_ex.c\
stm32f4xx_hal_qspi.c\
stm32f4xx_hal_rcc.c\
stm32f4xx_hal_rcc_ex.c\
stm32f4xx_hal_rng.c\
stm32f4xx_hal_rtc.c\
stm32f4xx_hal_rtc_ex.c\
stm32f4xx_hal_sai.c\
stm32f4xx_hal_sai_ex.c\
stm32f4xx_hal_sd.c\
stm32f4xx_hal_sdram.c\
stm32f4xx_hal_smartcard.c\
stm32f4xx_hal_spdifrx.c\
stm32f4xx_hal_spi.c\
stm32f4xx_hal_sram.c\
stm32f4xx_hal_tim.c\
stm32f4xx_hal_timebase_tim_template.c\
stm32f4xx_hal_tim_ex.c\
stm32f4xx_hal_uart.c\
stm32f4xx_hal_usart.c\
stm32f4xx_hal_wwdg.c\
stm32f4xx_ll_fmc.c\
stm32f4xx_ll_fsmc.c\
stm32f4xx_ll_sdmmc.c\
stm32f4xx_ll_usb.c\
stmpe1600.c\
stmpe811.c\
ts3510.c\
wm8994.c


OBJS = $(SRCS:%.c=$(BUILD_DIR)%.o)

all: libstm32f469_discovery.a

$(BUILD_DIR)%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libstm32f469_discovery.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f469_discovery.a

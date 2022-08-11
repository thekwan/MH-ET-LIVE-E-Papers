
# Put your stlink folder here so make burn will work.
STLINK=/home/thekwan/Work/STM32/stlink

vpath %.c  Src                                 \
	       Drivers/STM32F1xx_HAL_Driver/Src    \
	       User/Config                         \
	       User/e-Paper                        \
	       User/Fonts                          \
	       User/GUI                            \

# Put your source files here (or *.c, etc)
SRCS = main.c                      \
       gpio.c                      \
       spi.c                       \
       usart.c                     \
       stm32f1xx_hal_msp.c         \
       stm32f1xx_it.c              \
       system_stm32f1xx.c          \
       stm32f1xx_ll_adc.c          \
       stm32f1xx_ll_crc.c          \
       stm32f1xx_ll_dac.c          \
       stm32f1xx_ll_dma.c          \
       stm32f1xx_ll_exti.c         \
       stm32f1xx_ll_fsmc.c         \
       stm32f1xx_ll_gpio.c         \
       stm32f1xx_ll_i2c.c          \
       stm32f1xx_ll_pwr.c          \
       stm32f1xx_ll_rcc.c          \
       stm32f1xx_ll_rtc.c          \
       stm32f1xx_ll_sdmmc.c        \
       stm32f1xx_ll_spi.c          \
       stm32f1xx_ll_tim.c          \
       stm32f1xx_ll_usart.c        \
       stm32f1xx_ll_usb.c          \
       stm32f1xx_ll_utils.c        \
       stm32f1xx_hal.c             \
       stm32f1xx_hal_adc.c         \
       stm32f1xx_hal_can.c         \
       stm32f1xx_hal_cec.c         \
       stm32f1xx_hal_cortex.c      \
       stm32f1xx_hal_crc.c         \
       stm32f1xx_hal_dac.c         \
       stm32f1xx_hal_dma.c         \
       stm32f1xx_hal_eth.c         \
       stm32f1xx_hal_gpio.c        \
       stm32f1xx_hal_hcd.c         \
       stm32f1xx_hal_i2c.c         \
       stm32f1xx_hal_i2s.c         \
       stm32f1xx_hal_irda.c        \
       stm32f1xx_hal_iwdg.c        \
       stm32f1xx_hal_mmc.c         \
       stm32f1xx_hal_nand.c        \
       stm32f1xx_hal_nor.c         \
       stm32f1xx_hal_pccard.c      \
       stm32f1xx_hal_pcd.c         \
       stm32f1xx_hal_pwr.c         \
       stm32f1xx_hal_rcc.c         \
       stm32f1xx_hal_rtc.c         \
       stm32f1xx_hal_sd.c          \
       stm32f1xx_hal_spi.c         \
       stm32f1xx_hal_sram.c        \
       stm32f1xx_hal_tim.c         \
       stm32f1xx_hal_uart.c        \
       stm32f1xx_hal_usart.c       \
       stm32f1xx_hal_wwdg.c        \
       DEV_Config.c                \
       EPD_1in54b.c                \
       ImageData.c                 \
       font12.c                    \
       font16.c                    \
       font20.c                    \
       font24.c                    \
       font8.c                     \
       GUI_Paint.c

       #stm32f1xx_hal_timebase_rtc_alarm_template.c  \
       #stm32f1xx_hal_timebase_tim_template.c        \
       #stm32f1xx_hal_msp_template.c\
       #stm32f1xx_hal_flash.c \

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=epaper

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

LDFLAGS = -TDrivers/STM32F103RBTx_FLASH.ld
CFLAGS += -lc -lm -lnosys
CFLAGS += -mcpu=cortex-m3
CFLAGS += -mthumb
CFLAGS += -mfloat-abi=soft
CFLAGS += -mlittle-endian
CFLAGS += -fdata-sections
CFLAGS += -ffunction-sections
CFLAGS += -Wall
CFLAGS += -Werror
CFLAGS += -Warray-bounds
CFLAGS += -fno-strict-aliasing
CFLAGS += -Wno-unused-const-variable
#CFLAGS += -specs=nano.specs
CFLAGS += -specs=nosys.specs
CFLAGS += -Os
CFLAGS += -g
CFLAGS += -DUSE_FULL_LL_DRIVER
#CFLAGS += -DUSE_FULL_ASSERT
CFLAGS += -DSTM32F103xB

# Include files from STM libraries
CFLAGS += -IDrivers/STM32F1xx_HAL_Driver/Inc
CFLAGS += -IDrivers/CMSIS/Device/ST/STM32F1xx/Include
CFLAGS += -IDrivers/CMSIS/Include
CFLAGS += -IUser/Config
CFLAGS += -IUser/e-Paper
CFLAGS += -IUser/Fonts
CFLAGS += -IUser/GUI
CFLAGS += -IInc

# only_one_usart_mode
#CFLAGS += -DONLY_ONE_USART_MODE

# DEBUG ENABLE
#CFLAGS += -DDEBUG_ENABLE

###################################################

# Includes
# ..
###################################################
OBJDIR=objs
OBJS=$(patsubst %,$(OBJDIR)/%,$(SRCS:.c=.o))

# add startup file to build
#OBJS = $(SRCS:.c=.o)

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(OBJDIR)/%.o: %.c
	@mkdir -p $(OBJDIR)
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo 'CC $@'

$(PROJ_NAME).elf: $(OBJS)
	@echo 'LD $@'
	@$(CC) $(CFLAGS) -o $@ $^ Drivers/startup_stm32f103xb.s $(LDFLAGS)
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -rf *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin $(OBJDIR)

# Flash the STM32F4
burn: proj
	st-flash write $(PROJ_NAME).bin 0x8000000

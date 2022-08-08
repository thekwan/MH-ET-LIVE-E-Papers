
# Put your stlink folder here so make burn will work.
STLINK=/home/thekwan/Work/STM32/stlink

# Put your source files here (or *.c, etc)
SRCS = Src/main.c                      \
       Src/gpio.c                      \
       Src/spi.c                       \
       Src/usart.c                     \
       Src/stm32f1xx_hal_msp.c         \
       Src/stm32f1xx_it.c              \
       Src/system_stm32f1xx.c
 

# Library code
STM32CUBE_DIR=.
SRCS += ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_adc.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_crc.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dac.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c   \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_fsmc.c   \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c   \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_i2c.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rtc.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_sdmmc.c  \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_spi.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_tim.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usart.c  \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c    \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c  \
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c       \
        ${STM32CUBE_DIR}/User/Config/DEV_Config.c                               \
        ${STM32CUBE_DIR}/User/e-Paper/EPD_1in54b.c                              \
        ${STM32CUBE_DIR}/User/e-Paper/ImageData.c                               \
        ${STM32CUBE_DIR}/User/Fonts/font12.c                                    \
        ${STM32CUBE_DIR}/User/Fonts/font16.c                                    \
        ${STM32CUBE_DIR}/User/Fonts/font20.c                                    \
        ${STM32CUBE_DIR}/User/Fonts/font24.c                                    \
        ${STM32CUBE_DIR}/User/Fonts/font8.c                                     \
        ${STM32CUBE_DIR}/User/GUI/GUI_Paint.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=epaper

# Put your STM32F4 library code directory here
#STM_COMMON=/home/thekwan/Work/STM32/Projects/STM32Cube_FW_F4_V1.10.0
#STM_COMMON=../../../../..

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

#CFLAGS  = -T./SW4STM32/STM32F103RB-Nucleo/STM32F103RBTx_FLASH.ld
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
CFLAGS += -O0
CFLAGS += -g
CFLAGS += -DUSE_FULL_LL_DRIVER
#CFLAGS += -DUSE_FULL_ASSERT
CFLAGS += -DSTM32F103xB

# Include files from STM libraries
CFLAGS += -I${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
CFLAGS += -I${STM32CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
CFLAGS += -I${STM32CUBE_DIR}/Drivers/CMSIS/Include
CFLAGS += -I${STM32CUBE_DIR}/User/Config
CFLAGS += -I${STM32CUBE_DIR}/User/e-Paper
CFLAGS += -I${STM32CUBE_DIR}/User/Fonts
CFLAGS += -I${STM32CUBE_DIR}/User/GUI
CFLAGS += -IInc

# only_one_usart_mode
#CFLAGS += -DONLY_ONE_USART_MODE

# DEBUG ENABLE
#CFLAGS += -DDEBUG_ENABLE

###################################################

# Includes
# ..
###################################################


# add startup file to build
#OBJS = $(SRCS:.c=.o)

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

#$(PROJ_NAME).elf: $(OBJS)
$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32F4
burn: proj
	st-flash write $(PROJ_NAME).bin 0x8000000

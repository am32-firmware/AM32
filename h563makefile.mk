
TARGETS_H563 := $(call get_targets,H563)

HAL_FOLDER_H563 := $(HAL_FOLDER)/h563

MCU_H563 := -mcpu=cortex-m33 -mthumb 
LDSCRIPT_H563 := $(HAL_FOLDER_H563)/STM32H563AGI6_FLASH.ld



SRC_DIR_H563_STARTUP := $(HAL_FOLDER_H563)/Startup
SRC_DIR_H563_SRC := $(HAL_FOLDER_H563)/Src
SRC_DIR_H563_HAL :=	$(HAL_FOLDER_H563)/Drivers/STM32H5xx_HAL_Driver/Src

SRC_DIR_H563 := \
	$(SRC_DIR_H563_STARTUP) \
	$(SRC_DIR_H563_SRC) \
	$(SRC_DIR_H563_HAL)

CFLAGS_H563 := \
	-I$(HAL_FOLDER_H563)/Inc \
	-I$(HAL_FOLDER_H563)/Drivers/STM32H5xx_HAL_Driver/Inc \
	-I$(HAL_FOLDER_H563)/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_H563)/Drivers/CMSIS/Device/ST/STM32H5xx/Include

CFLAGS_H563 += \
	-DSTM32H563xx \
	-DUSE_FULL_LL_DRIVER


# SRC_H563 := $(foreach dir,$(SRC_DIR_H563),$(wildcard $(dir)/*.[cs]))

SRC_H563 := \
	$(SRC_DIR_H563_STARTUP)/startup_stm32h563xx.s \
	$(SRC_DIR_H563_SRC)/phaseouts.c \
	$(SRC_DIR_H563_SRC)/peripherals.c \
	$(SRC_DIR_H563_SRC)/IO.c \
	$(SRC_DIR_H563_SRC)/eeprom.c \
	$(SRC_DIR_H563_SRC)/comparator.c \
	$(SRC_DIR_H563_SRC)/ADC.c \
	$(SRC_DIR_H563_SRC)/spi.c \
	$(SRC_DIR_H563_SRC)/clock.c \
	$(SRC_DIR_H563_SRC)/flash.c \
	$(SRC_DIR_H563_SRC)/dma-stm32h5.c \
	$(SRC_DIR_H563_SRC)/exti-stm32h5.c \
	$(SRC_DIR_H563_SRC)/led-sk6812-spi.c \
	$(SRC_DIR_H563_SRC)/commutation-timer.c \
	$(SRC_DIR_H563_SRC)/ten-khz-timer.c \
	$(SRC_DIR_H563_SRC)/bridge.c \
	$(SRC_DIR_H563_SRC)/drv8323-spi.c \
	$(SRC_DIR_H563_SRC)/usart.c \
	$(SRC_DIR_H563_SRC)/gpio.c \
	$(SRC_DIR_H563_SRC)/power.c \
	$(SRC_DIR_H563_SRC)/serial_telemetry.c \
	$(SRC_DIR_H563_SRC)/mcu.c \
	$(SRC_DIR_H563_SRC)/stm32h5xx_it.c \
	$(SRC_DIR_H563_SRC)/system_stm32h5xx.c \
	$(SRC_DIR_H563_HAL)/stm32h5xx_ll_usart.c \
	$(SRC_DIR_H563_HAL)/stm32h5xx_ll_rcc.c \
	$(SRC_DIR_H563_HAL)/stm32h5xx_ll_gpio.c \
	$(SRC_DIR_H563_HAL)/stm32h5xx_ll_tim.c
	

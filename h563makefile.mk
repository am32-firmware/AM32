
TARGETS_H563 := $(call get_targets,H563)

HAL_FOLDER_H563 := $(HAL_FOLDER)/h563

MCU_H563 := -mcpu=cortex-m33 -mthumb 
LDSCRIPT_H563 := $(HAL_FOLDER_H563)/AT32H563x6_FLASH.ld

SRC_DIR_H563 := \
	$(HAL_FOLDER_H563)/Startup \
	$(HAL_FOLDER_H563)/Src \
	$(HAL_FOLDER_H563)/Drivers/drivers/src

CFLAGS_H563 := \
	-I$(HAL_FOLDER_H563)/Inc \
	-I$(HAL_FOLDER_H563)/Drivers/STM32H5xx_HAL_Driver/Inc \
	-I$(HAL_FOLDER_H563)/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_H563)/Drivers/CMSIS/Device/ST/STM32H5xx/Include

CFLAGS_H563 += \
	 -DSTM32H563xx \
	 -DUSE_STDPERIPH_DRIVER


SRC_H563 := $(foreach dir,$(SRC_DIR_H563),$(wildcard $(dir)/*.[cs]))

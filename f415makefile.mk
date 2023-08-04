
TARGETS_F415 := \
	AT32DEV_F415 TEKKO32_F415

HAL_FOLDER_F415 := $(HAL_FOLDER)/F415

MCU_F415 := -mcpu=cortex-m4 -mthumb
LDSCRIPT_F415 := $(HAL_FOLDER_F415)/AT32F415x8_FLASH.ld

SRC_DIR_F415 := \
	$(HAL_FOLDER_F415)/Startup \
	$(HAL_FOLDER_F415)/Src \
	$(HAL_FOLDER_F415)/Drivers/drivers/src

CFLAGS_F415 := \
	-I$(HAL_FOLDER_F415)/Inc \
	-I$(HAL_FOLDER_F415)/Drivers/drivers/inc \
	-I$(HAL_FOLDER_F415)/Drivers/CMSIS/cm4/core_support \
	-I$(HAL_FOLDER_F415)/Drivers/CMSIS/cm4/device_support

CFLAGS_F415 += \
	 -DAT32F415K8U7_4 \
	 -DUSE_STDPERIPH_DRIVER
	

SRC_F415 := $(foreach dir,$(SRC_DIR_F415),$(wildcard $(dir)/*.[cs]))

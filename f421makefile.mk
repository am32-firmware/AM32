
TARGETS_F421 := $(call get_targets,F421)

HAL_FOLDER_F421 := $(HAL_FOLDER)/f421

MCU_F421 := -mcpu=cortex-m4 -mthumb 
LDSCRIPT_F421 := $(HAL_FOLDER_F421)/AT32F421x6_FLASH.ld

SRC_DIR_F421 := \
	$(HAL_FOLDER_F421)/Startup \
	$(HAL_FOLDER_F421)/Src \
	$(HAL_FOLDER_F421)/Drivers/drivers/src

CFLAGS_F421 := \
	-I$(HAL_FOLDER_F421)/Inc \
	-I$(HAL_FOLDER_F421)/Drivers/drivers/inc \
	-I$(HAL_FOLDER_F421)/Drivers/CMSIS/cm4/core_support \
	-I$(HAL_FOLDER_F421)/Drivers/CMSIS/cm4/device_support

CFLAGS_F421 += \
	 -DAT32F421K8U7 \
	 -DUSE_STDPERIPH_DRIVER


SRC_F421 := $(foreach dir,$(SRC_DIR_F421),$(wildcard $(dir)/*.[cs]))

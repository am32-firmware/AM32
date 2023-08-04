
TARGETS_E230 := GD32DEV_A_E230\
	

HAL_FOLDER_E230 := $(HAL_FOLDER)/e230

MCU_E230 := -mcpu=cortex-m23 -mthumb -fsigned-char
LDSCRIPT_E230 := $(HAL_FOLDER_E230)/GD32E230K8_FLASH.ld

SRC_DIR_E230 := \
	$(HAL_FOLDER_E230)/Src \
	$(HAL_FOLDER_E230)/Drivers/CMSIS/Source\
	$(HAL_FOLDER_E230)/Drivers/GD32E23x_standard_peripheral/Source \
	$(HAL_FOLDER_E230)/Startup 

CFLAGS_E230 += \
	-I$(HAL_FOLDER_E230)/Inc \
	-I$(HAL_FOLDER_E230)/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_E230)/Drivers/CMSIS/Core/Include \
	-I$(HAL_FOLDER_E230)/Drivers/GD32E23x_standard_peripheral/Include 

CFLAGS_E230 += \
	-DGD32E230 \
    -DGD32E23x \
	-DUSE_STDPERIPH_DRIVER


SRC_E230 := $(foreach dir,$(SRC_DIR_E230),$(wildcard $(dir)/*.[cs]))

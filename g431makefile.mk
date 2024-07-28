MCU := G431
PART := STM32G431xx

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/g431

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/STM32G4xx_HAL_Driver/Src

SRC_DIR_$(MCU) := \
	$(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/STM32G4xx_HAL_Driver/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Device/ST/STM32G4xx/Include

CFLAGS_$(MCU) += \
	-DHSE_VALUE=8000000 \
	-D$(PART) \
	-DHSE_STARTUP_TIMEOUT=100 \
	-DLSE_STARTUP_TIMEOUT=5000 \
	-DUSE_FULL_LL_DRIVER

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cs]))
SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(HAL_FOLDER_$(MCU))/Src/eeprom.c \
	$(HAL_FOLDER_$(MCU))/Src/system_stm32g4xx.c \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*_it.c)

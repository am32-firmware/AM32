MCU := L431
PART := STM32L431xx

TARGETS_$(MCU) := \
    VIMDRONES_$(MCU) 

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(call lc,$(MCU))

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb -g -gdwarf-2
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/STM32L4xx_HAL_Driver/Src

SRC_DIR_$(MCU) := \
	$(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/STM32L4xx_HAL_Driver/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Device/ST/STM32L4xx/Include

CFLAGS_$(MCU) += \
	-DHSE_VALUE=8000000 \
	-D$(PART) \
	-DHSE_STARTUP_TIMEOUT=100 \
	-DLSE_STARTUP_TIMEOUT=5000 \
	-DLSE_VALUE=32768 \
	-DDATA_CACHE_ENABLE=1 \
	-DINSTRUCTION_CACHE_ENABLE=0 \
	-DVDD_VALUE=3300 \
	-DLSI_VALUE=32000 \
	-DHSI_VALUE=16000000 \
	-DUSE_FULL_LL_DRIVER \
	-DPREFETCH_ENABLE=1

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cs]))
SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(HAL_FOLDER_$(MCU))/Src/eeprom.c \
	$(HAL_FOLDER_$(MCU))/Src/system_stm32l4xx.c \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*_it.c)

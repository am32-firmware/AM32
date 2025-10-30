MCU := G431
PART := STM32G431xx

MCU_LC := $(call lc,$(MCU))

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb
LDSCRIPT_$(MCU) := $(HAL_FOLDER_$(MCU))/ldscript.ld

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup/gcc \
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
	-DUSE_FULL_LL_DRIVER \
	-DPREFETCH_ENABLE=1

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cs]))

# optional CAN support
CFLAGS_CAN_$(MCU) = \
	-ISrc/DroneCAN \
	-ISrc/DroneCAN/libcanard \
	-ISrc/DroneCAN/dsdl_generated/include

SRC_DIR_CAN_$(MCU) = Src/DroneCAN \
		Src/DroneCAN/dsdl_generated/src \
		Src/DroneCAN/libcanard

SRC_CAN_$(MCU) := $(foreach dir,$(SRC_DIR_CAN_$(MCU)),$(wildcard $(dir)/*.[cs]))

LDSCRIPT_CAN_$(MCU) := $(HAL_FOLDER_$(MCU))/ldscript_CAN.ld

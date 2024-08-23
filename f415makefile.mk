MCU := F415
PART := AT32F415K8U7_4

MCU_LC := $(call lc,$(MCU))

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb
LDSCRIPT_$(MCU) := $(HAL_FOLDER_$(MCU))/ldscript.ld

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/drivers/src

SRC_DIR_$(MCU) := $(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/drivers/inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/cm4/core_support \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/cm4/device_support

CFLAGS_$(MCU) += \
	 -D$(PART) \
	 -DUSE_STDPERIPH_DRIVER

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

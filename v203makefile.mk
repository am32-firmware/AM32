MCU := V203
PART := CH32V203

RISCV_SDK_PREFIX := tools/$(OSDIR)/riscv-embedded-gcc/bin/riscv-none-embed-
$(MCU)_CC := $(RISCV_SDK_PREFIX)gcc
$(MCU)_OBJCOPY := $(RISCV_SDK_PREFIX)objcopy

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(call lc,$(MCU))

MCU_$(MCU) := -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -msave-restore -nostartfiles -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -DMCU_FLASH_START=0x08000000
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Drivers/Core \
	$(HAL_FOLDER_$(MCU))/Drivers/Peripheral/src \
	$(HAL_FOLDER_$(MCU))/Drivers/Debug \
	$(HAL_FOLDER_$(MCU))/Startup

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/Peripheral/inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/Core \
	-I$(HAL_FOLDER_$(MCU))/Drivers/Debug

SRC_DIR_$(MCU) := \
	$(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src

CFLAGS_$(MCU) += \
	-D$(PART)

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cS]))

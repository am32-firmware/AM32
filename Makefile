
QUIET = @

# tools
CC = $(ARM_SDK_PREFIX)gcc
OBJCOPY = $(ARM_SDK_PREFIX)objcopy
ECHO = echo

# common variables
IDENTIFIER := AM32

# Folders
HAL_FOLDER := Mcu
MAIN_SRC_DIR := Src
MAIN_INC_DIR := Inc

SRC_DIRS_COMMON := $(MAIN_SRC_DIR)

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# include the rules for OS independence
include $(ROOT)/make/tools.mk

# supported MCU types
MCU_TYPES := E230 F031 F051 F415 F421 G071
MCU_TYPE := NONE

# Function to include makefile for each MCU type
define INCLUDE_MCU_MAKEFILES
$(foreach MCU_TYPE,$(MCU_TYPES),$(eval include $(call lc,$(MCU_TYPE))makefile.mk))
endef
$(call INCLUDE_MCU_MAKEFILES)

# additional libs
LIBS := -lnosys

# extract version from Inc/version.h
VERSION_MAJOR := $(shell $(FGREP) "define VERSION_MAJOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
VERSION_MINOR := $(shell $(FGREP) "define VERSION_MINOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )

FIRMWARE_VERSION := $(VERSION_MAJOR).$(VERSION_MINOR)

# Compiler options

CFLAGS_BASE := -DUSE_MAKE -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_BASE += -I$(MAIN_INC_DIR) -g3 -O3 -Wall -ffunction-sections

CFLAGS_COMMON := $(CFLAGS_BASE) -D$(TARGET)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Search source files
SRC_COMMON := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

TARGET_FNAME = $(IDENTIFIER)_$(TARGET)_$(FIRMWARE_VERSION)
TARGET_BASENAME = $(BIN_DIR)/$(TARGET_FNAME)

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

.PHONY : clean all binary $(foreach MCU,$(MCU_TYPES),$(call lc,$(MCU)))
ALL_TARGETS := $(foreach MCU,$(MCU_TYPES),$(TARGETS_$(MCU)))
all : $(ALL_TARGETS)

# create targets for compiling one mcu type, eg "make f421"
define CREATE_TARGET
$(call lc,$(1)) : $$(TARGETS_$(1))
endef
$(foreach MCU,$(MCU_TYPES),$(eval $(call CREATE_TARGET,$(MCU))))

clean :
	@echo Removing $(OBJ) directory
	@$(RM) -rf $(OBJ)


binary : $(TARGET_BASENAME).bin
# we copy debug.elf to give us a constant debug target for vscode
# this means the debug button will always debug the last target built
	@$(CP) -f $(OBJ)$(DSEP)$(TARGET_FNAME).elf $(OBJ)$(DSEP)debug.elf > $(NUL)
# also copy the openocd.cfg from the MCU directory to obj/openocd.cfg for auto config of Cortex-Debug
# in vscode
	@$(CP) -f Mcu$(DSEP)$(call lc,$(MCU_TYPE))$(DSEP)openocd.cfg $(OBJ)$(DSEP)openocd.cfg > $(NUL)
	@$(ECHO) done $(TARGET)

# create targets compiling each MCU types targets
define CREATE_MCU_TARGETS
$$(TARGETS_$(1)) :
	@$$(MAKE) -s MCU_TYPE=$(1) TARGET=$$@ binary
endef
$(foreach MCU,$(MCU_TYPES),$(eval $(call CREATE_MCU_TARGETS,$(MCU))))


# Compile target
$(TARGET_BASENAME).elf: CFLAGS := $(MCU_$(MCU_TYPE)) $(CFLAGS_$(MCU_TYPE)) $(CFLAGS_COMMON)
$(TARGET_BASENAME).elf: LDFLAGS := $(LDFLAGS_COMMON) $(LDFLAGS_$(MCU_TYPE)) -T$(LDSCRIPT_$(MCU_TYPE))
$(TARGET_BASENAME).elf: $(SRC_COMMON) $(SRC_$(MCU_TYPE))
	@$(ECHO) Compiling $(notdir $@)
	$(QUIRT)$(MKDIR) -p $(OBJ)
	$(QUIET)$(CC) $(CFLAGS) $(LDFLAGS) -MMD -MP -MF $(@:.elf=.d) -o $(@) $(SRC_COMMON) $(SRC_$(MCU_TYPE))

# Generate bin and hex files
$(TARGET_BASENAME).bin: $(TARGET_BASENAME).elf
	@$(ECHO) Generating $(notdir $@)
	$(QUIET)$(OBJCOPY) -O binary $(<) $@
	$(QUIET)$(OBJCOPY) $(<) -O ihex $(@:.bin=.hex)

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

define SHOW_MCU_TARGETS
$(1) Targets $(TARGETS_$(1))\n
endef

targets:
	$(QUIET)echo List of targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(ALL_TARGETS)

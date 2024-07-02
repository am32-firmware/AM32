
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
MCU_TYPES := E230 F031 F051 F415 F421 G071 L431 G431 V203
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

CFLAGS_BASE := -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_BASE += -I$(MAIN_INC_DIR) -g3 -O2 -ffunction-sections --specs=nosys.specs
CFLAGS_BASE += -Wall -Wundef -Wextra -Werror -Wno-unused-parameter -Wno-stringop-truncation

CFLAGS_COMMON := $(CFLAGS_BASE)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Search source files
SRC_COMMON := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

# Function to check for _CAN suffix
has_can_suffix = $(findstring _CAN,$1)

# find the SVD files
$(foreach MCU,$(MCU_TYPES),$(eval SVD_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.svd)))

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

#####################
# main firmware build
define CREATE_BUILD_TARGET
$(2)_BASENAME = $(BIN_DIR)/$(IDENTIFIER)_$(2)_$(FIRMWARE_VERSION)

$(2) : $$($(2)_BASENAME).bin

# get MCU specific compiler, objcopy and link script or use the ARM SDK one
$(eval xCC := $(if $($(MCU)_CC), $($(MCU)_CC), $(CC)))
$(eval xOBJCOPY := $(if $($(MCU)_OBJCOPY), $($(MCU)_OBJCOPY), $(OBJCOPY)))

# Generate bin and hex files from elf
$$($(2)_BASENAME).bin: $$($(2)_BASENAME).elf
	echo building BIN $$@
	@$(ECHO) Generating $$(notdir $$@)
	$(QUIET)$(xOBJCOPY) -O binary $$(<) $$@
	$(QUIET)python3 Src/DroneCAN/set_app_signature.py $$@ $$(<)
	$(QUIET)$(xOBJCOPY) $$(<) -O ihex $$(@:.bin=.hex)
	$(QUIET)$(CP) -f $$(<) $(OBJ)$(DSEP)debug.elf > $(NUL)

# check for CAN support
$(eval xLDSCRIPT := $$(if $$(call has_can_suffix,$$(2)),$(LDSCRIPT_CAN_$(1)),$(LDSCRIPT_$(1))))
$(eval xCFLAGS := $$(if $$(call has_can_suffix,$$(2)),$(CFLAGS_CAN_$(1))))
$(eval xSRC := $$(if $$(call has_can_suffix,$$(2)),$(SRC_CAN_$(1))))

CFLAGS_$(2) = -DAM32_MCU=\"$(MCU)\" $(MCU_$(1)) -D$(2) $(CFLAGS_$(1)) $(CFLAGS_COMMON) $(xCFLAGS)
LDFLAGS_$(2) = $(LDFLAGS_COMMON) $(LDFLAGS_$(1)) -T$(xLDSCRIPT)

-include $$($(2)_BASENAME).d

$$($(2)_BASENAME).elf: $(SRC_COMMON) $$(SRC_$(1)) $(xSRC)
	@$(ECHO) Compiling $$(notdir $$@)
	$(QUIET)$(MKDIR) -p $(OBJ)
	$(QUIET)$(xCC) $$(CFLAGS_$(2)) $$(LDFLAGS_$(2)) -MMD -MP -MF $$(@:.elf=.d) -o $$(@) $(SRC_COMMON) $$(SRC_$(1)) $(xSRC)
# we copy debug.elf to give us a constant debug target for vscode
# this means the debug button will always debug the last target built
	$(QUIET)$(CP) -f $$(SVD_$(1)) $(OBJ)/debug.svd
# also copy the openocd.cfg from the MCU directory to obj/openocd.cfg for auto config of Cortex-Debug
# in vscode
	$(QUIET)$(CP) -f Mcu$(DSEP)$(call lc,$(1))$(DSEP)openocd.cfg $(OBJ)$(DSEP)openocd.cfg > $(NUL)
endef
$(foreach MCU,$(MCU_TYPES),$(foreach TARGET,$(TARGETS_$(MCU)), $(eval $(call CREATE_BUILD_TARGET,$(MCU),$(TARGET)))))

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

# useful target to list all of the board targets so you can see what
# make target to use for your board
targets:
	$(QUIET)echo List of targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(ALL_TARGETS)


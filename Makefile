
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

# Include processor specific makefiles
include f051makefile.mk
include g071makefile.mk
include f031makefile.mk
include f421makefile.mk
include e230makefile.mk
include f415makefile.mk
include gd32makefile.mk

# Default MCU type to F051
MCU_TYPE ?= F051

# additional libs
LIBS := -lnosys

# extract version from Inc/version.h
VERSION_MAJOR := $(shell $(FGREP) "define VERSION_MAJOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
VERSION_MINOR := $(shell $(FGREP) "define VERSION_MINOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )

FIRMWARE_VERSION := $(VERSION_MAJOR).$(VERSION_MINOR)

# Compiler options
CFLAGS_COMMON := -DUSE_MAKE -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_COMMON += -I$(MAIN_INC_DIR) -g -O3 -Wall -ffunction-sections
CFLAGS_COMMON += -D$(TARGET)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Search source files
SRC_COMMON := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

TARGET_FNAME = $(IDENTIFIER)_$(TARGET)_$(FIRMWARE_VERSION)
TARGET_BASENAME = $(BIN_DIR)/$(TARGET_FNAME)

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

TOOLS_DIR ?= $(ROOT)/tools
DL_DIR := $(ROOT)/downloads

.PHONY : clean all binary f051 g071 f031 e230 f421 f415
ALL_TARGETS := $(TARGETS_F051) $(TARGETS_G071) $(TARGETS_F031) $(TARGETS_E230) $(TARGETS_F421) $(TARGETS_F415)
all : $(ALL_TARGETS)
f051 : $(TARGETS_F051)
g071 : $(TARGETS_G071)
f031 : $(TARGETS_F031)
e230 : $(TARGETS_E230)
f421 : $(TARGETS_F421)
f415 : $(TARGETS_F415)

clean :
	@echo Removing $(OBJ) directory
	@$(RM) -rf $(OBJ)

# lowercase version of MCU_TYPE
MCU_LOWER = $(call lc,$(MCU_TYPE))

binary : $(TARGET_BASENAME).bin
# we copy debug.elf to give us a constant debug target for vscode
# this means the debug button will always debug the last target built
	@$(CP) -f $(OBJ)$(DSEP)$(TARGET_FNAME).elf $(OBJ)$(DSEP)debug.elf > $(NUL)
# also copy the openocd.cfg from the MCU directory to obj/openocd.cfg for auto config of Cortex-Debug
# in vscode
	@$(CP) -f Mcu$(DSEP)$(MCU_LOWER)$(DSEP)openocd.cfg $(OBJ)$(DSEP)openocd.cfg > $(NUL)
	@$(ECHO) done $(TARGET)

$(TARGETS_F051) :
	@$(MAKE) -s MCU_TYPE=F051 TARGET=$@ binary

$(TARGETS_G071) :
	@$(MAKE) -s MCU_TYPE=G071 TARGET=$@ binary

$(TARGETS_F031) :
	@$(MAKE) -s MCU_TYPE=F031 TARGET=$@ binary

$(TARGETS_E230) :
	@$(MAKE) -s MCU_TYPE=E230 TARGET=$@ binary

$(TARGETS_F421) :
	@$(MAKE) -s MCU_TYPE=F421 TARGET=$@ binary	

$(TARGETS_F415) :
	@$(MAKE) -s MCU_TYPE=F415 TARGET=$@ binary		

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

# mkdirs
$(DL_DIR):
	$(QUIET)$(MKDIR) -p $@

$(TOOLS_DIR):
	$(QUIET)$(MKDIR) -p $@

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

targets:
	$(QUIET)echo Targets for each MCU. To build a target use 'make TARGETNAME'
	$(QUIET)echo F051 Targets:  $(TARGETS_F051)
	$(QUIET)echo G071 Targets: $(TARGETS_G071)
	$(QUIET)echo F031 Targets: $(TARGETS_F031)
	$(QUIET)echo E230 Targets: $(TARGETS_E230)
	$(QUIET)echo F421 Targets: $(TARGETS_F421)
	$(QUIET)echo F415 Targets: $(TARGETS_F415)
	$(QUIET)echo GD32 Targets: $(TARGETS_GD32)

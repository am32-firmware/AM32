QUIET = @

# tools
CC = $(ARM_SDK_PREFIX)gcc
CP = $(ARM_SDK_PREFIX)objcopy
ECHO = echo

# common variables
IDENTIFIER := AM32

# Folders
HAL_FOLDER := Mcu
MAIN_SRC_DIR := Src
MAIN_INC_DIR := Inc

SRC_DIRS_COMMON := $(MAIN_SRC_DIR)

# Include processor specific makefiles
include f051makefile.mk
include g071makefile.mk
include f031makefile.mk
include f421makefile.mk
include e230makefile.mk
include f415makefile.mk
include l431makefile.mk

# Default MCU type to F051
MCU_TYPE ?= F051

# additional libs
LIBS := -lnosys

# Compiler options
CFLAGS_COMMON := -DUSE_MAKE -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_COMMON += -I$(MAIN_INC_DIR) -g -O3 -Wall -ffunction-sections
CFLAGS_COMMON += -D$(TARGET)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# Search source files
SRC_COMMON := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

VERSION_MAJOR := $(shell grep "#define VERSION_MAJOR" $(MAIN_SRC_DIR)/main.c | cut -d' ' -f3 )
VERSION_MINOR := $(shell grep "#define VERSION_MINOR" $(MAIN_SRC_DIR)/main.c | cut -d' ' -f3 )
CFLAGS_COMMON += -DVERSION_MAJOR=$(VERSION_MAJOR) -DVERSION_MINOR=$(VERSION_MINOR)

FIRMWARE_VERSION := $(VERSION_MAJOR).$(VERSION_MINOR)

TARGET_BASENAME = $(BIN_DIR)/$(IDENTIFIER)_$(TARGET)_$(FIRMWARE_VERSION)

# Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(ROOT)/make/system-id.mk

# configure some directories that are relative to wherever ROOT_DIR is located
BIN_DIR := $(ROOT)/obj

TOOLS_DIR ?= $(ROOT)/tools
DL_DIR := $(ROOT)/downloads

.PHONY : clean all binary f051 g071 f031 l431
all : $(TARGETS_F051) $(TARGETS_G071) $(TARGETS_F031) $(TARGETS_L431)
f051 : $(TARGETS_F051)
g071 : $(TARGETS_G071)
f031 : $(TARGETS_F031)
l431 : $(TARGETS_L431)

clean :
	rm -rf $(BIN_DIR)/*

binary : $(TARGET_BASENAME).bin
	@$(ECHO) done $(TARGET)

# include the DroneCAN makefile
ifeq ($(MCU_TYPE),L431)
include $(ROOT)/make/DroneCAN.mk
endif

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

$(TARGETS_L431) :
	@$(MAKE) -s MCU_TYPE=L431 TARGET=$@ binary

# Compile target
$(TARGET_BASENAME).elf: SRC := $(SRC_COMMON) $(SRC_DRONECAN) $(SRC_$(MCU_TYPE))
$(TARGET_BASENAME).elf: CFLAGS := $(MCU_$(MCU_TYPE)) $(CFLAGS_$(MCU_TYPE)) $(CFLAGS_COMMON) $(CFLAGS_DRONECAN)
$(TARGET_BASENAME).elf: LDFLAGS := $(LDFLAGS_COMMON) $(LDFLAGS_$(MCU_TYPE)) -T$(LDSCRIPT_$(MCU_TYPE))
$(TARGET_BASENAME).elf: $(SRC)
	@$(ECHO) Compiling $(notdir $@)
	$(QUIET)mkdir -p $(dir $@)
	$(QUIET)$(CC) $(CFLAGS) $(LDFLAGS) -MMD -MP -MF $(@:.elf=.d) -o $(@) $(SRC)

# Generate bin and hex files
$(TARGET_BASENAME).bin: $(TARGET_BASENAME).elf
	@$(ECHO) Generating $(notdir $@)
	$(QUIET)$(CP) -O binary $(<) $@
	$(QUIET)$(CP) $(<) -O ihex $(@:.bin=.hex)

# mkdirs
$(DL_DIR):
	$(QUIET)mkdir -p $@

$(TOOLS_DIR):
	$(QUIET)mkdir -p $@

# include the tools makefile
include $(ROOT)/make/tools.mk

targets:
	$(QUIET)echo "Targets for each MCU. To build a target use 'make TARGETNAME'"
	$(QUIET)echo "F051 Targets: " $(TARGETS_F051)
	$(QUIET)echo "G071 Targets: " $(TARGETS_G071)
	$(QUIET)echo "F031 Targets: " $(TARGETS_F031)
	$(QUIET)echo "E230 Targets: " $(TARGETS_E230)
	$(QUIET)echo "F421 Targets: " $(TARGETS_F421)
	$(QUIET)echo "F415 Targets: " $(TARGETS_F415)
	$(QUIET)echo "L431 Targets: " $(TARGETS_L431)

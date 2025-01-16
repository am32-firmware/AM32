
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
include h563makefile.mk

# Default MCU type to F051
MCU_TYPE ?= F051

# additional libs
LIBS := -lnosys

# extract version from Inc/version.h
VERSION_MAJOR := $(shell $(FGREP) "define VERSION_MAJOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
VERSION_MINOR := $(shell $(FGREP) "define VERSION_MINOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )

FIRMWARE_VERSION := $(VERSION_MAJOR).$(VERSION_MINOR)

# Compiler options
# CFLAGS_COMMON := -DUSE_MAKE -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_COMMON := -DUSE_MAKE -fsingle-precision-constant -fomit-frame-pointer
CFLAGS_COMMON += -I$(MAIN_INC_DIR) -g -Wall -ffunction-sections -Wno-comment
CFLAGS_COMMON += -D$(TARGET)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=output.map

# Search source files
SRC_COMMON := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))
SRC_MAIN := $(SRC_DIRS_COMMON)/main.c
SRC_COMMON := $(filter-out $(SRC_MAIN), $(SRC_COMMON))
$(info    $(SRC_COMMON))

# SRC_MAIN := $(SRC_DIRS_COMMON)/main.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/t ests/test-adc.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-as5048-debug.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-as5048-read.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-as5048-led.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-as5048-set-zero.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-as5048-usart.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-input-pwm.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-spi-gpio.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-spi.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-spi-neopixel.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-spi-neopixel2.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-uart-tx.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-aux-uart-sk6812.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-bridge-audio.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-bridge-commutate.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-can-io.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-comparator-3phase.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-comparator.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-debug.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-delay.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-drv8323.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-drv8323-spi-read.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-eeprom.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-eeprom-erase.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-eeprom-read.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-eeprom-write.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-exti-button.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-exti-button-led.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-exti-3phase.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-exti-comparator-3phase.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-gpio-high.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-gpio-low.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-input-pwm.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi2.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi3.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi4.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi5.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi6.c
SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-sk6812-spi7.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-led-maxfrequency.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-nothing.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-rng.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-phaseouts.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-spi-aux-as5048-read.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-spi-dma-single.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-spi-dma.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-ten-khz-timer.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-usart-aux.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-usart-debug.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-usart-main.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-usart-main-rs485.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-vreg-5V.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-watchdog-fail.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-watchdog-pass.c
# SRC_MAIN := $(SRC_DIRS_COMMON)/tests/test-watchdog-period.c

TARGET_FNAME = $(IDENTIFIER)_$(TARGET)_$(FIRMWARE_VERSION)
TARGET_BASENAME = $(BIN_DIR)/$(TARGET_FNAME)

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

TOOLS_DIR ?= $(ROOT)/tools
DL_DIR := $(ROOT)/downloads

.PHONY : clean all binary f051 g071 f031 e230 f421 f415 h563
ALL_TARGETS := $(TARGETS_F051) $(TARGETS_G071) $(TARGETS_F031) $(TARGETS_E230) $(TARGETS_F421) $(TARGETS_F415) $(TARGETS_H563)
all : $(ALL_TARGETS)
f051 : $(TARGETS_F051)
g071 : $(TARGETS_G071)
f031 : $(TARGETS_F031)
e230 : $(TARGETS_E230)
f421 : $(TARGETS_F421)
f415 : $(TARGETS_F415)
h563 : $(TARGETS_H563)

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

$(TARGETS_H563) :
	@$(MAKE) -s MCU_TYPE=H563 TARGET=$@ binary

# Compile target
$(TARGET_BASENAME).elf: CFLAGS := $(MCU_$(MCU_TYPE)) $(CFLAGS_$(MCU_TYPE)) $(CFLAGS_COMMON)
$(TARGET_BASENAME).elf: LDFLAGS := $(LDFLAGS_COMMON) $(LDFLAGS_$(MCU_TYPE)) -T$(LDSCRIPT_$(MCU_TYPE))
$(TARGET_BASENAME).elf: $(SRC_MAIN) $(SRC_COMMON) $(SRC_$(MCU_TYPE))
	@$(ECHO) Compiling $(notdir $@)
	$(QUIET)$(MKDIR) -p $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -MMD -MP -MF $(@:.elf=.d) -o $(@) $(SRC_MAIN) $(SRC_COMMON) $(SRC_$(MCU_TYPE))

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
	$(QUIET)echo H563 Targets: $(TARGETS_H563)

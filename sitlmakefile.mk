MCU := SITL

MCU_LC := $(call lc,$(MCU))

ifeq ($(OS),Windows_NT)
ifeq ($(UNAME_O),Cygwin)
TARGETS_$(MCU) := $(call get_targets,$(MCU))
else
# plain Windows (cmd.exe or git-bash/MinGW): no POSIX environment, the
# SITL only builds under Cygwin there
TARGETS_$(MCU) :=
endif
else
TARGETS_$(MCU) := $(call get_targets,$(MCU))
endif

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU)

# native build using the host compiler
SITL_CC := gcc
SITL_OBJCOPY := objcopy
NATIVE_$(MCU) := 1

MCU_$(MCU) :=
LDSCRIPT_$(MCU) :=

SRC_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Src \
	$(HAL_FOLDER_$(MCU))/sim

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/sim

CFLAGS_$(MCU) += -D_GNU_SOURCE
# newlib based hosts (Cygwin) have no __WORDSIZE for the canard.h default
CFLAGS_$(MCU) += -DCANARD_64_BIT="(__SIZEOF_POINTER__ == 8)"

# native compiler flags, replacing the ARM specific CFLAGS_COMMON. Inc is
# searched via -iquote rather than -I so that Inc/signal.h does not shadow
# the system <signal.h>
ifeq ($(UNAME_S),Darwin)
# clang: no -fsingle-precision-constant, and ignore the gcc-only
# -Wno- options inherited from the common CFLAGS
SITL_GCC_FLAGS := -Wno-unknown-warning-option
else
SITL_GCC_FLAGS := -fsingle-precision-constant -Wno-stringop-truncation
endif
# optional sanitizers for the native build. SITL_SANITIZE=address builds
# with AddressSanitizer (also enable LeakSanitizer at run time); other
# useful values: undefined, thread, address,undefined. Off by default so
# normal and CI builds are unaffected:
#   make AM32_SITL_CAN SITL_SANITIZE=address
# ASan needs the frame pointer and its own optimisation level; a
# sanitized build is not bit-comparable with a normal one and runs
# slower, so it is a debugging build, not a release
ifneq ($(SITL_SANITIZE),)
SITL_SAN_FLAGS := -fsanitize=$(SITL_SANITIZE) -fno-omit-frame-pointer -O1
else
SITL_SAN_FLAGS :=
endif

# gcov/gcovr code coverage of the firmware under the SITL:
#   make AM32_SITL_CAN SITL_COVERAGE=1
# emits .gcno next to the objects and .gcda at run time; kept at the
# normal -O2 so the instrumented SITL still runs at full speed (gcov
# maps optimised code fine). A coverage build is not a normal build
ifeq ($(SITL_COVERAGE),1)
SITL_COV_FLAGS := --coverage -fno-omit-frame-pointer -DSITL_COVERAGE
else
SITL_COV_FLAGS :=
endif

# -funsigned-char matches the ARM targets, where char is unsigned
CFLAGS_COMMON_$(MCU) := $(SITL_GCC_FLAGS) -funsigned-char -iquote $(MAIN_INC_DIR) -g3 -O2 \
	-Wall -Wundef -Wextra -Werror -Wno-unused-parameter \
	-fno-strict-aliasing -pthread $(SITL_SAN_FLAGS) $(SITL_COV_FLAGS)

LDFLAGS_COMMON_$(MCU) := -pthread $(SITL_SAN_FLAGS) $(SITL_COV_FLAGS)

LDLIBS_$(MCU) := -lm

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.c))

# optional CAN support
CFLAGS_CAN_$(MCU) = \
	-ISrc/DroneCAN \
	-ISrc/DroneCAN/libcanard \
	-ISrc/DroneCAN/dsdl_generated/include

SRC_DIR_CAN_$(MCU) = Src/DroneCAN \
		Src/DroneCAN/dsdl_generated/src \
		Src/DroneCAN/libcanard

SRC_CAN_$(MCU) := $(foreach dir,$(SRC_DIR_CAN_$(MCU)),$(wildcard $(dir)/*.[cs]))

LDSCRIPT_CAN_$(MCU) :=

# ==============================================================================
# AM32 FIRMWARE BUILD SYSTEM
# ==============================================================================
cmake_minimum_required(VERSION 3.20)

# Official Release Mode: If ON, targets in TARGET_RELEASE_SKIP_LIST will be skipped
option(AM32_OFFICIAL_RELEASE "If ON, targets in TARGET_RELEASE_SKIP_LIST will be skipped" OFF)

# Dependency Management (Downloads GCC/OpenOCD if missing)
include(cmake/manage_tools.cmake)

# Permanently Broken / Invalid Targets (Never Build)
set(TARGET_SKIP_LIST
    "DAKEFPV_35A_F415")

# Private / Test Targets (Build in CI, Skip in Official Release)
set(TARGET_RELEASE_SKIP_LIST    
    "LMNR_SIEGE_100A8S_SINGLE_G071"
    "LMNR_SIEGE_80A8S_4IN1_G071"
    "VORTEX_100A_F421"
    "VORTEX_80A_F421"
)

# TOOLCHAIN SELECTION
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    if(ARCH STREQUAL "ARM")
        set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/arm-none-eabi.cmake")
    elseif(ARCH STREQUAL "WCH_RISCV")
        set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/wch-riscv-none-embed.cmake")
    else()
        message(FATAL_ERROR "No ARCH specified! usage: cmake -DARCH=ARM (or WCH_RISCV) ..")
    endif()
endif()

# PROJECT INITIALIZATION
project(AM32_${ARCH} C ASM)

# MODULE LOADING
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
include(VersionUtils)
include(TargetFactory)

message(STATUS "[${ARCH}] Configuring AM32 v${FIRMWARE_VERSION} (${FIRMWARE_GIT_HASH})")

# LOAD TARGETS
if(ARCH STREQUAL "ARM")
    file(GLOB TARGETS "${CMAKE_SOURCE_DIR}/cmake/targets/arm/*.cmake")
elseif(ARCH STREQUAL "WCH_RISCV")
    file(GLOB TARGETS "${CMAKE_SOURCE_DIR}/cmake/targets/wch_riscv/*.cmake")
endif()

foreach(T ${TARGETS})
    message(STATUS "Loading Target: ${T}")
    include(${T})
endforeach()

# IDE CONFIGURATION (Local Builds Only)
if(NOT AM32_OFFICIAL_RELEASE)
    include(VscodeGenerator)
    am32_generate_launch_json()
endif()

message(STATUS "--- Configuration Complete ---")
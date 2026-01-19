set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Skip Linker Check (Fixes the "exit undefined" error)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Point to the tools (Relative to this file)
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(MY_GCC_ROOT "${PROJECT_ROOT}/tools/gcc-arm")

if(EXISTS "${MY_GCC_ROOT}")
    set(CMAKE_PROGRAM_PATH ${MY_GCC_ROOT}/bin ${CMAKE_PROGRAM_PATH})
else()
    message(WARNING "ARM GCC not found at ${MY_GCC_ROOT}. Build might fail.")
endif()

# Find Compilers
find_program(CMAKE_C_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++)
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)
find_program(CMAKE_SIZE arm-none-eabi-size)

# Lock down search modes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Set SysRoot for Intellisense
if(CMAKE_C_COMPILER)
    execute_process(
        COMMAND ${CMAKE_C_COMPILER} -print-sysroot
        OUTPUT_VARIABLE ARM_SYSROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    # Setting this variable makes CMake add --sysroot to all compile commands automatically
    set(CMAKE_SYSROOT "${ARM_SYSROOT}")
endif()

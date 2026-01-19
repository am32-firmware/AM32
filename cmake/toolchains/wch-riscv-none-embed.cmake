set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
# FIX: Now pointing to the WCH specific directory
set(MY_GCC_ROOT "${PROJECT_ROOT}/tools/gcc-wch-riscv")

if(EXISTS "${MY_GCC_ROOT}")
    set(CMAKE_PROGRAM_PATH ${MY_GCC_ROOT}/bin ${CMAKE_PROGRAM_PATH})
else()
    message(WARNING "WCH RISC-V GCC not found at ${MY_GCC_ROOT}. Build might fail.")
endif()

find_program(CMAKE_C_COMPILER NAMES riscv-none-elf-gcc riscv-none-embed-gcc)
find_program(CMAKE_CXX_COMPILER NAMES riscv-none-elf-g++ riscv-none-embed-g++)
find_program(CMAKE_ASM_COMPILER NAMES riscv-none-elf-gcc riscv-none-embed-gcc)
find_program(CMAKE_OBJCOPY NAMES riscv-none-elf-objcopy riscv-none-embed-objcopy)
find_program(CMAKE_SIZE NAMES riscv-none-elf-size riscv-none-embed-size)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Sysroot for IntelliSense
if(CMAKE_C_COMPILER)
    execute_process(
        COMMAND ${CMAKE_C_COMPILER} -print-sysroot
        OUTPUT_VARIABLE WCH_RISCV_SYSROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(CMAKE_SYSROOT "${WCH_RISCV_SYSROOT}")
endif()
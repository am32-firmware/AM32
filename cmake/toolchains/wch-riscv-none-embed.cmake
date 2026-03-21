set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)


if(DEFINED ENV{AM32_WCH_TOOLCHAIN_PATH})
    set(TOOLCHAIN_PARENT "$ENV{AM32_WCH_TOOLCHAIN_PATH}")
    set(TOOLCHAIN_ROOT "$ENV{AM32_WCH_TOOLCHAIN_PATH}/Toolchain/RISC-V Embedded GCC")
    message(STATUS "[Toolchain] Using Environment Variable: ${TOOLCHAIN_ROOT}")
else() #Default Local Tools (Linux/Mac Automated Download)
    get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
    set(TOOLCHAIN_ROOT "${PROJECT_ROOT}/tools/gcc-wch-riscv")
endif()

set(TOOLCHAIN_BIN_PATH "${TOOLCHAIN_ROOT}/bin")

# Validation
if(EXISTS "${TOOLCHAIN_ROOT}")
    # Prepend to CMAKE_PROGRAM_PATH so find_program prefers this location
    list(PREPEND CMAKE_PROGRAM_PATH "${TOOLCHAIN_BIN_PATH}")
else()
    if(DEFINED AM32_WCH_TOOLCHAIN_PATH OR DEFINED ENV{AM32_WCH_TOOLCHAIN_PATH})
        message(FATAL_ERROR "WCH Toolchain path was specified but does not exist!\nPath: ${TOOLCHAIN_ROOT}")
    else()
        message(WARNING "WCH RISC-V GCC not found at default location: '${TOOLCHAIN_ROOT}'.\nIf on Windows, ensure AM32_WCH_TOOLCHAIN_PATH environment variable is set.")
    endif()
endif()

# FIND COMPILERS
find_program(CMAKE_C_COMPILER NAMES riscv-none-elf-gcc riscv-none-embed-gcc HINTS "${TOOLCHAIN_BIN_PATH}" NO_CACHE)
find_program(CMAKE_CXX_COMPILER NAMES riscv-none-elf-g++ riscv-none-embed-g++ HINTS "${TOOLCHAIN_BIN_PATH}" NO_CACHE)
find_program(CMAKE_ASM_COMPILER NAMES riscv-none-elf-gcc riscv-none-embed-gcc HINTS "${TOOLCHAIN_BIN_PATH}" NO_CACHE)
find_program(CMAKE_OBJCOPY NAMES riscv-none-elf-objcopy riscv-none-embed-objcopy HINTS "${TOOLCHAIN_BIN_PATH}" NO_CACHE)
find_program(CMAKE_SIZE NAMES riscv-none-elf-size riscv-none-embed-size HINTS "${TOOLCHAIN_BIN_PATH}" NO_CACHE)
find_program(CMAKE_OBJDUMP NAMES riscv-none-elf-objdump riscv-none-embed-objdump HINTS "${TOOLCHAIN_BIN_PATH}" NO_CACHE)


# Determine where OpenOCD might be relative to GCC
if(NOT AM32_OPENOCD_EXECUTABLE)
    find_program(AM32_OPENOCD_EXECUTABLE
        NAMES openocd
        HINTS 
            "${TOOLCHAIN_PARENT}/OpenOCD/OpenOCD/bin"   # MounRiver Studio Layout
            "${TOOLCHAIN_PARENT}/openocd-wch-riscv/bin" # Automated Download Layout
        NO_CACHE
    )

    if(AM32_OPENOCD_EXECUTABLE)
        message(STATUS "[Toolchain] Found OpenOCD: ${AM32_OPENOCD_EXECUTABLE}")
    else()
        message(STATUS "[Toolchain] Warning: OpenOCD not found near GCC. Flashing may require manual path configuration.")
    endif()
endif()


# CROSS COMPILATION SETTINGS
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
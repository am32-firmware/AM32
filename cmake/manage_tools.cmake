cmake_minimum_required(VERSION 3.20)

# 1. SETUP DIRECTORIES
set(TOOLS_DIR "${CMAKE_CURRENT_LIST_DIR}/../tools")
get_filename_component(TOOLS_DIR "${TOOLS_DIR}" ABSOLUTE)
set(DOWNLOADS_DIR "${CMAKE_CURRENT_LIST_DIR}/../downloads")
file(MAKE_DIRECTORY "${TOOLS_DIR}")
file(MAKE_DIRECTORY "${DOWNLOADS_DIR}")

# 2. ARM GCC
set(ARM_GCC_DIR "${TOOLS_DIR}/gcc-arm")
set(ARM_URL_BASE "https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major")

if(CMAKE_HOST_WIN32)
    set(ARM_URL "${ARM_URL_BASE}-win32.zip")
    set(ARM_CACHE_FILE "${DOWNLOADS_DIR}/arm-gcc-win32.zip")
elseif(CMAKE_HOST_APPLE)
    set(ARM_URL "${ARM_URL_BASE}-mac.tar.bz2")
    set(ARM_CACHE_FILE "${DOWNLOADS_DIR}/arm-gcc-mac.tar.bz2")
else()
    set(ARM_URL "${ARM_URL_BASE}-x86_64-linux.tar.bz2")
    set(ARM_CACHE_FILE "${DOWNLOADS_DIR}/arm-gcc-linux.tar.bz2")
endif()

if(NOT EXISTS "${ARM_GCC_DIR}")
    message(STATUS "[Tools] ARM GCC missing. Installing...")
    if(NOT EXISTS "${ARM_CACHE_FILE}")
        file(DOWNLOAD "${ARM_URL}" "${ARM_CACHE_FILE}" SHOW_PROGRESS)
    endif()
    file(ARCHIVE_EXTRACT INPUT "${ARM_CACHE_FILE}" DESTINATION "${TOOLS_DIR}")
    file(GLOB EXTRACTED "${TOOLS_DIR}/gcc-arm-none-eabi-*")
    if(EXTRACTED)
        file(RENAME "${EXTRACTED}" "${ARM_GCC_DIR}")
    endif()
else()
    message(STATUS "[Tools] ARM GCC present.")
endif()

# 3. ARM OPENOCD (xPack)
set(ARM_OPENOCD_DIR "${TOOLS_DIR}/openocd-arm")
set(XP_BASE "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-3/xpack-openocd-0.12.0-3")

if(CMAKE_HOST_WIN32)
    set(XP_URL "${XP_BASE}-win32-x64.zip")
    set(XP_CACHE "${DOWNLOADS_DIR}/openocd-win.zip")
elseif(CMAKE_HOST_APPLE)
    set(XP_URL "${XP_BASE}-darwin-x64.tar.gz")
    set(XP_CACHE "${DOWNLOADS_DIR}/openocd-mac.tar.gz")
else()
    set(XP_URL "${XP_BASE}-linux-x64.tar.gz")
    set(XP_CACHE "${DOWNLOADS_DIR}/openocd-linux.tar.gz")
endif()

if(NOT EXISTS "${ARM_OPENOCD_DIR}")
    message(STATUS "[Tools] ARM OpenOCD missing. Installing...")
    if(NOT EXISTS "${XP_CACHE}")
        file(DOWNLOAD "${XP_URL}" "${XP_CACHE}" SHOW_PROGRESS)
    endif()
    file(ARCHIVE_EXTRACT INPUT "${XP_CACHE}" DESTINATION "${TOOLS_DIR}")
    file(GLOB XP_EXTRACTED "${TOOLS_DIR}/xpack-openocd-*")
    if(XP_EXTRACTED)
        file(RENAME "${XP_EXTRACTED}" "${ARM_OPENOCD_DIR}")
    endif()
else()
    message(STATUS "[Tools] ARM OpenOCD present.")
endif()

# ==============================================================================
# 4. WCH (MOUNRIVER) RISC-V TOOLCHAIN
# ==============================================================================
set(WCH_GCC_DIR "${TOOLS_DIR}/gcc-wch-riscv") 
set(WCH_OCD_DIR "${TOOLS_DIR}/openocd-wch-riscv")
set(WCH_CACHE_FILE "${DOWNLOADS_DIR}/wch_toolchain.archive")

if(NOT EXISTS "${WCH_GCC_DIR}" OR NOT EXISTS "${WCH_OCD_DIR}")
    message(STATUS "[Tools] WCH RISC-V Tools missing. Installing...")

    # 1. Download (Same as before)
    if(NOT EXISTS "${WCH_CACHE_FILE}")
        if(CMAKE_HOST_WIN32)
            set(MRS_ID "1823552948062416898")
        elseif(CMAKE_HOST_APPLE)
            set(MRS_ID "1991353229960581122")
        else()
            set(MRS_ID "1993872528621211649")
        endif()
        
        set(API_URL "https://api.mounriver.com/mountriver/api/version/fetchRecentOpenOcdUrl?resourceId=${MRS_ID}")
        file(DOWNLOAD "${API_URL}" "${DOWNLOADS_DIR}/mrs.json")
        file(READ "${DOWNLOADS_DIR}/mrs.json" JSON)
        string(REGEX MATCH "\"result\":[ \t]*\"([^\"]+)\"" _ ${JSON})
        set(DL_URL ${CMAKE_MATCH_1})
        file(REMOVE "${DOWNLOADS_DIR}/mrs.json")
        
        message(STATUS "[Tools] Downloading WCH Toolchain...")
        file(DOWNLOAD "${DL_URL}" "${WCH_CACHE_FILE}" SHOW_PROGRESS)
    endif()

    message(STATUS "[Tools] Extracting WCH Toolchain...")
    
    # 2. Extract
    set(TEMP_ROOT "${TOOLS_DIR}/riscv_temp_extract")
    if(EXISTS "${TEMP_ROOT}")
        file(REMOVE_RECURSE "${TEMP_ROOT}")
    endif()
    file(MAKE_DIRECTORY "${TEMP_ROOT}")
    file(ARCHIVE_EXTRACT INPUT "${WCH_CACHE_FILE}" DESTINATION "${TEMP_ROOT}")

    # 3. Smart Search for GCC (Recursive, depth-agnostic)
    #    We search for the binary name anywhere.
    file(GLOB_RECURSE GCC_FOUND "${TEMP_ROOT}/riscv-none-embed-gcc*")
    
    set(FINAL_GCC_BIN "")
    foreach(ITEM ${GCC_FOUND})
        # Filter: Ensure it is inside a 'bin' folder to avoid picking up unrelated files
        if(ITEM MATCHES "/bin/riscv-none-embed-gcc")
            set(FINAL_GCC_BIN "${ITEM}")
            break()
        endif()
    endforeach()

    if(FINAL_GCC_BIN)
        get_filename_component(GCC_BIN_DIR "${FINAL_GCC_BIN}" DIRECTORY)
        get_filename_component(GCC_ROOT "${GCC_BIN_DIR}/.." ABSOLUTE)
        
        if(EXISTS "${WCH_GCC_DIR}") 
            file(REMOVE_RECURSE "${WCH_GCC_DIR}")
        endif()
        file(RENAME "${GCC_ROOT}" "${WCH_GCC_DIR}")
    else()
        message(FATAL_ERROR "[Tools] Failed to locate 'riscv-none-embed-gcc' in extracted archive!")
    endif()


    # 4. Smart Search for OpenOCD
    #    We search for the binary name anywhere.
    file(GLOB_RECURSE OCD_FOUND "${TEMP_ROOT}/openocd*")
    
    set(FINAL_OCD_BIN "")
    foreach(ITEM ${OCD_FOUND})
        # Filter: Ensure it is inside 'bin' and ends with 'openocd' (or .exe)
        # We avoid 'openocd.cfg' or folder names
        if(ITEM MATCHES "/bin/openocd(\\.exe)?$")
            set(FINAL_OCD_BIN "${ITEM}")
            break()
        endif()
    endforeach()

    if(FINAL_OCD_BIN)
        get_filename_component(OCD_BIN_DIR "${FINAL_OCD_BIN}" DIRECTORY)
        get_filename_component(OCD_ROOT "${OCD_BIN_DIR}/.." ABSOLUTE)
        
        if(EXISTS "${WCH_OCD_DIR}") 
            file(REMOVE_RECURSE "${WCH_OCD_DIR}")
        endif()
        file(RENAME "${OCD_ROOT}" "${WCH_OCD_DIR}")
    else()
        message(FATAL_ERROR "[Tools] Failed to locate 'openocd' in extracted archive!")
    endif()

    # Cleanup
    file(REMOVE_RECURSE "${TEMP_ROOT}")

else()
    message(STATUS "[Tools] WCH RISC-V Tools present.")
endif()
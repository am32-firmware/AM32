# ==============================================================================
# MODULE: TargetFactory
# ==============================================================================
include(TargetUtils)

if(NOT Python3_EXECUTABLE)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()

# Find Common Sources ONCE for the whole project
# These files (Src/*.c) are used by every single target, so we shouldn't scan disk 500 times.
file(GLOB AM32_COMMON_SOURCES "Src/*.c" "Src/*.s" "Src/*.S")
set(AM32_COMMON_INCLUDES Inc)

# ------------------------------------------------------------------------------
# MACRO: Process Family Targets (THE MAIN ITERATOR)
# ------------------------------------------------------------------------------
macro(am32_process_family)
    set(options "")
    # Added LINKER_SCRIPT and CAN_LINKER_SCRIPT back as required/optional arguments
    set(oneValueArgs FAMILY ARCH LINKER_SCRIPT CAN_LINKER_SCRIPT SVD_FILE OCD_CONFIG_FILE) 
    set(multiValueArgs SOURCE_DIRS CAN_SOURCE_DIRS INCLUDES CAN_INCLUDES DEFINES CPU_FLAGS)

    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # 1. Expand Directories to Source Files (Here, once per family)
    set(FAMILY_SOURCES "")
    _am32_add_sources_from_dirs(FAMILY_SOURCES "${ARG_SOURCE_DIRS}")

    set(FAMILY_CAN_SOURCES "")
    _am32_add_sources_from_dirs(FAMILY_CAN_SOURCES "${ARG_CAN_SOURCE_DIRS}")

    # 2. Get Boards
    get_target_boards(BOARD_LIST "${ARG_FAMILY}")

    foreach(BOARD_NAME ${BOARD_LIST})

        # ALWAYS SKIP: Broken or disabled targets
        if("${BOARD_NAME}" IN_LIST TARGET_SKIP_LIST)
            message(STATUS "[${ARG_FAMILY}] Skipping broken target: ${BOARD_NAME}")
            continue()
        endif()

        #  RELEASE SKIP: Private/Test targets (Only if AM32_OFFICIAL_RELEASE is ON)
        if(AM32_OFFICIAL_RELEASE AND "${BOARD_NAME}" IN_LIST TARGET_RELEASE_SKIP_LIST)
            message(STATUS "[${ARG_FAMILY}] Skipping non-release target: ${BOARD_NAME}")
            continue()
        endif()

        add_am32_target(
            BOARD_ID          "${BOARD_NAME}"
            MCU_FAMILY        "${ARG_FAMILY}"
            ARCH              "${ARG_ARCH}"
            
            LINKER_SCRIPT     "${ARG_LINKER_SCRIPT}"
            CAN_LINKER_SCRIPT "${ARG_CAN_LINKER_SCRIPT}"

            SVD_FILE          "${ARG_SVD_FILE}"
            OCD_CONFIG_FILE   "${ARG_OCD_CONFIG_FILE}"
            CPU_FLAGS         ${ARG_CPU_FLAGS}
            SOURCES           ${FAMILY_SOURCES}      # Pass expanded files
            CAN_SOURCES       ${FAMILY_CAN_SOURCES}  # Pass expanded files
            INCLUDES          ${ARG_INCLUDES}
            CAN_INCLUDES      ${ARG_CAN_INCLUDES}
            DEFINES           ${ARG_DEFINES} ${BOARD_NAME}
        )

        message(STATUS "TARGET:${BOARD_NAME}")
    endforeach()
endmacro()

# ------------------------------------------------------------------------------
# FUNCTION: add_am32_target (THE BUILDER)
# ------------------------------------------------------------------------------
function(add_am32_target)
    cmake_parse_arguments(ARG "" "BOARD_ID;MCU_FAMILY;LINKER_SCRIPT;CAN_LINKER_SCRIPT;ARCH;SVD_FILE;OCD_CONFIG_FILE" "DEFINES;INCLUDES;SOURCES;CAN_SOURCES;CAN_INCLUDES;CPU_FLAGS" ${ARGN})

    if(NOT "${ARG_ARCH}" STREQUAL "${ARCH}")
        return()
    endif()

    # Naming
    set(LOGICAL_TARGET "${ARG_BOARD_ID}")
    set(OUTPUT_FILENAME "AM32_${LOGICAL_TARGET}_${FIRMWARE_VERSION}")
    set(ELF_FILE "${OUTPUT_FILENAME}.elf")

    # Sources
    set(ALL_SOURCES ${AM32_COMMON_SOURCES} ${ARG_SOURCES})
    set(IS_CAN_TARGET FALSE)
    
    if(LOGICAL_TARGET MATCHES "_CAN")
        set(IS_CAN_TARGET TRUE)
        list(APPEND ALL_SOURCES ${ARG_CAN_SOURCES})
    endif()

    add_executable(${LOGICAL_TARGET} ${ALL_SOURCES})
    set_target_properties(${LOGICAL_TARGET} PROPERTIES OUTPUT_NAME "${OUTPUT_FILENAME}" SUFFIX ".elf")

    # Compiler Setup
    set(COMMON_COMPILE_FLAGS
        -fsingle-precision-constant -fomit-frame-pointer -ffast-math -g3 -O3
        -ffunction-sections -Wall -Wundef -Wextra -Werror -Wno-unused-parameter
        -Wno-stringop-truncation
        # Maps the absolute source directory to "." in debug symbols. Strips off local file system paths - reproducible builds
        -fdebug-prefix-map="${CMAKE_SOURCE_DIR}=."
        # Allow to set __DATE__ & __TIME__ - used in VersionUtils.cmake
        -Wno-builtin-macro-redefined 
    )
    set(COMMON_LINK_FLAGS --specs=nosys.specs --specs=nano.specs -lnosys -Wl,--gc-sections -Wl,--print-memory-usage)

    target_compile_definitions(${LOGICAL_TARGET} PRIVATE 
        AM32_MCU="${ARG_MCU_FAMILY}"
        ${ARG_DEFINES}
        FIRMWARE_GIT_HASH="${FIRMWARE_GIT_HASH}"
        FIRMWARE_VERSION="${FIRMWARE_VERSION}"
    )
    target_compile_options(${LOGICAL_TARGET} PRIVATE ${ARG_CPU_FLAGS} ${COMMON_COMPILE_FLAGS})

    # Linker
    if(IS_CAN_TARGET)
        if(ARG_CAN_LINKER_SCRIPT)
            set(FINAL_LDSCRIPT ${ARG_CAN_LINKER_SCRIPT})
        else()
            # Fail if the Target File didn't provide a CAN script
            message(FATAL_ERROR "[${LOGICAL_TARGET}] CAN target requested, but CAN_LINKER_SCRIPT was not defined in target_${ARG_MCU_FAMILY}.cmake!")
        endif()
    else()
        if(ARG_LINKER_SCRIPT)
            set(FINAL_LDSCRIPT ${ARG_LINKER_SCRIPT})
        else()
             message(FATAL_ERROR "[${LOGICAL_TARGET}] LINKER_SCRIPT was not defined in target_${ARG_MCU_FAMILY}.cmake!")
        endif()
    endif()

    target_link_options(${LOGICAL_TARGET} PRIVATE ${ARG_CPU_FLAGS} ${COMMON_LINK_FLAGS} -T${FINAL_LDSCRIPT} "-Wl,-Map=${OUTPUT_FILENAME}.map")
    target_include_directories(${LOGICAL_TARGET} PRIVATE ${AM32_COMMON_INCLUDES} ${ARG_INCLUDES} $<$<BOOL:${IS_CAN_TARGET}>:${ARG_CAN_INCLUDES}>)

    # Post-Build
    set(BANNER_MSG "================ [ ${OUTPUT_FILENAME} ] ================")

    if(IS_CAN_TARGET)
        add_custom_command(TARGET ${LOGICAL_TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O binary ${ELF_FILE} ${OUTPUT_FILENAME}.bin
            COMMAND Python3::Interpreter ${CMAKE_SOURCE_DIR}/Src/DroneCAN/set_app_signature.py ${OUTPUT_FILENAME}.bin ${ELF_FILE}
            COMMAND ${CMAKE_OBJCOPY} ${ELF_FILE} -O ihex ${OUTPUT_FILENAME}.hex

            COMMAND ${CMAKE_COMMAND} -E echo "${BANNER_MSG}"
            COMMAND ${CMAKE_SIZE} ${ELF_FILE}
        )
    else()
        add_custom_command(TARGET ${LOGICAL_TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O binary ${ELF_FILE} ${OUTPUT_FILENAME}.bin
            COMMAND ${CMAKE_OBJCOPY} ${ELF_FILE} -O ihex ${OUTPUT_FILENAME}.hex

            COMMAND ${CMAKE_COMMAND} -E echo "${BANNER_MSG}"
            COMMAND ${CMAKE_SIZE} ${ELF_FILE}
        )
    endif()

    # Flash Target
    if(ARG_OCD_CONFIG_FILE)
        if(CMAKE_HOST_WIN32)
            set(EXE_EXT ".exe")
        else()
            set(EXE_EXT "")
        endif()

        # We manually construct the path because we know where manage_tools.cmake put it
        if(ARG_ARCH STREQUAL "WCH_RISCV")
            set(TOOL_OPENOCD "${CMAKE_SOURCE_DIR}/tools/openocd-wch-riscv/bin/openocd${EXE_EXT}")
        else()
            set(TOOL_OPENOCD "${CMAKE_SOURCE_DIR}/tools/openocd-arm/bin/openocd${EXE_EXT}")
        endif()

        add_custom_target(flash_${LOGICAL_TARGET}
            COMMAND ${TOOL_OPENOCD} -f ${ARG_OCD_CONFIG_FILE} -c "program ${ELF_FILE} verify reset exit"
            DEPENDS ${LOGICAL_TARGET}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            USES_TERMINAL
            COMMENT "Flashing ${LOGICAL_TARGET}..."
        )
    endif()
endfunction()
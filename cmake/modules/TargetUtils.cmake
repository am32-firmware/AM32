# ==============================================================================
# UTILITIES: Helpers for File finding and Text Parsing
# ==============================================================================

function(get_target_boards OUTPUT_VAR MCU_SUFFIX)
    set(BOARD_LIST "")
    set(TARGETS_FILE "${CMAKE_SOURCE_DIR}/Inc/targets.h")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${TARGETS_FILE}")

    if(NOT EXISTS "${TARGETS_FILE}")
        return()
    endif()

    file(READ "${TARGETS_FILE}" FILE_CONTENT)
    string(REPLACE ";" "\\;" FILE_CONTENT "${FILE_CONTENT}")
    string(REPLACE "\n" ";" FILE_LINES "${FILE_CONTENT}")

    foreach(LINE ${FILE_LINES})
        if("${LINE}" MATCHES "#define[ \t]+FILE_NAME" 
            AND NOT "${LINE}" MATCHES "^[ \t]*//"
            AND NOT "${LINE}" MATCHES "DISABLE_BUILD")
            
            if("${LINE}" MATCHES "_${MCU_SUFFIX}")
                string(REGEX MATCH "\"([^\"]+)\"" MATCHED_QUOTES "${LINE}")
                if(CMAKE_MATCH_1)
                    list(APPEND BOARD_LIST "${CMAKE_MATCH_1}")
                endif()
            endif()
        endif()
    endforeach()
    set(${OUTPUT_VAR} "${BOARD_LIST}" PARENT_SCOPE)
endfunction()


# ------------------------------------------------------------------------------
# MACRO: Find Linker Scripts
# ------------------------------------------------------------------------------
macro(am32_find_linker_scripts VAR_NORMAL VAR_CAN ROOT_DIR)
    file(GLOB _ALL_LDS "${CMAKE_SOURCE_DIR}/${ROOT_DIR}/*.ld")
    
    # Standard Script (prefer ldscript.ld, else first non-CAN)
    list(FIND _ALL_LDS "${CMAKE_SOURCE_DIR}/${ROOT_DIR}/ldscript.ld" _STD_IDX)
    if(_STD_IDX GREATER -1)
        list(GET _ALL_LDS ${_STD_IDX} ${VAR_NORMAL})
    else()
        foreach(_LD ${_ALL_LDS})
            if(NOT "${_LD}" MATCHES "_CAN.ld$")
                set(${VAR_NORMAL} "${_LD}")
                break()
            endif()
        endforeach()
    endif()

    if(NOT ${VAR_NORMAL})
        message(FATAL_ERROR "No Linker Script found in ${ROOT_DIR}")
    endif()

    # CAN Script
    set(_CAN_CANDIDATE "${CMAKE_SOURCE_DIR}/${ROOT_DIR}/ldscript_CAN.ld")
    if(EXISTS "${_CAN_CANDIDATE}")
        set(${VAR_CAN} "${_CAN_CANDIDATE}")
    else()
        set(${VAR_CAN} "")
    endif()
endmacro()

# ------------------------------------------------------------------------------
# INTERNAL HELPER: Add Sources From Directories
# ------------------------------------------------------------------------------
macro(_am32_add_sources_from_dirs OUTPUT_VAR DIR_LIST)
    foreach(DIR ${DIR_LIST})
        file(GLOB TEMP_SOURCES "${DIR}/*.c")
        list(APPEND ${OUTPUT_VAR} ${TEMP_SOURCES})
        file(GLOB TEMP_ASM "${DIR}/*.s" "${DIR}/*.S")
        list(APPEND ${OUTPUT_VAR} ${TEMP_ASM})
    endforeach()
endmacro()
set(MCU_ROOT "Mcu/f415")
set(MCU_PART "AT32F415K8U7_4")

am32_process_family(
    FAMILY            "F415"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/ldscript.ld"
    CAN_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/ldscript_CAN.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/AT32F415xx_v2.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS         -mcpu=cortex-m4 -mthumb
    
    # PASS DIRECTORIES, NOT FILES
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/drivers/src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    CAN_SOURCE_DIRS
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/dsdl_generated/src"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard"

    INCLUDES
        ${MCU_ROOT}/Inc
        ${MCU_ROOT}/Drivers/drivers/inc
        ${MCU_ROOT}/Drivers/CMSIS/cm4/core_support
        ${MCU_ROOT}/Drivers/CMSIS/cm4/device_support

    CAN_INCLUDES
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN/dsdl_generated/include

    DEFINES 
        ${MCU_PART} 
        F415
        USE_STDPERIPH_DRIVER
)
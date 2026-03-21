set(MCU_ROOT "Mcu/e230")
set(MCU_PART "GD32E230")

am32_process_family(
    FAMILY            "E230"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/GD32E230K8_FLASH.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/GD32E230.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS         -mcpu=cortex-m23 -mthumb -mfloat-abi=soft
    
    # PASS DIRECTORIES, NOT FILES
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/CMSIS/Source"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/GD32E23x_standard_peripheral/Source"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    INCLUDES
        ${MCU_ROOT}/Inc
        ${MCU_ROOT}/Drivers/CMSIS/Include
        ${MCU_ROOT}/Drivers/CMSIS/Core/Include
        ${MCU_ROOT}/Drivers/GD32E23x_standard_peripheral/Include

    DEFINES     
        ${MCU_PART}
        E230
        USE_STDPERIPH_DRIVER
)

set(MCU_ROOT "Mcu/f421")
set(MCU_PART "AT32F421K8U7")

am32_process_family(
    FAMILY            "F421"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/AT32F421x6_FLASH.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/AT32F421xx_v2.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS         -mcpu=cortex-m4 -mthumb
    
    # PASS DIRECTORIES, NOT FILES
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/drivers/src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    INCLUDES
        ${MCU_ROOT}/Inc
        ${MCU_ROOT}/Drivers/drivers/inc
        ${MCU_ROOT}/Drivers/CMSIS/cm4/core_support
        ${MCU_ROOT}/Drivers/CMSIS/cm4/device_support

    DEFINES     
        ${MCU_PART} 
        F421      
        USE_STDPERIPH_DRIVER
)

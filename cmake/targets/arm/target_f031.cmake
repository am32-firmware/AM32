set(MCU_ROOT "Mcu/f031")
set(MCU_PART "STM32F031x6")

am32_process_family(
    FAMILY            "F031"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/STM32F031C6TX_FLASH.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/STM32F0x1.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS         -mcpu=cortex-m0 -mthumb
    
    # PASS DIRECTORIES, NOT FILES
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    INCLUDES
        "${MCU_ROOT}/Inc"
        "${MCU_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc"
        "${MCU_ROOT}/Drivers/CMSIS/Include"
        "${MCU_ROOT}/Drivers/CMSIS/Device/ST/STM32F0xx/Include"

    DEFINES
        ${MCU_PART}    
        F031                                      
        USE_FULL_LL_DRIVER
        HSE_VALUE=8000000
        HSE_STARTUP_TIMEOUT=100
        LSE_STARTUP_TIMEOUT=5000
        LSE_VALUE=32768
        DATA_CACHE_ENABLE=0
        INSTRUCTION_CACHE_ENABLE=0
        VDD_VALUE=3300
        LSI_VALUE=40000
        HSI_VALUE=8000000
        PREFETCH_ENABLE=1
)

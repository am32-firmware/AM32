set(MCU_ROOT "Mcu/g031")
set(MCU_PART "STM32G031xx")

am32_process_family(
    FAMILY            "G031"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/STM32G031GBUX_FLASH.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/STM32G031.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS          -mcpu=cortex-m0plus -mthumb
    
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/STM32G0xx_HAL_Driver/Src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    INCLUDES
        ${MCU_ROOT}/Inc
        ${MCU_ROOT}/Drivers/STM32G0xx_HAL_Driver/Inc
        ${MCU_ROOT}/Drivers/CMSIS/Include
        ${MCU_ROOT}/Drivers/CMSIS/Device/ST/STM32G0xx/Include

    DEFINES
        ${MCU_PART}     
        G031                       
        USE_FULL_LL_DRIVER
        HSE_VALUE=8000000
        HSE_STARTUP_TIMEOUT=100
        LSE_STARTUP_TIMEOUT=5000
        LSE_VALUE=32768
        HSI_VALUE=16000000
        LSI_VALUE=32000
        VDD_VALUE=3300
        DATA_CACHE_ENABLE=1
        INSTRUCTION_CACHE_ENABLE=0
        PREFETCH_ENABLE=1
)

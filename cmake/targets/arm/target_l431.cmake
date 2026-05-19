set(MCU_ROOT "Mcu/l431")
set(MCU_PART "STM32L431xx")

am32_process_family(
    FAMILY            "L431"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/ldscript.ld"
    CAN_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/ldscript_CAN.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/STM32L4x1.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS
        -mcpu=cortex-m4 
        -mthumb
        -mfloat-abi=hard
    
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup/gcc"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/STM32L4xx_HAL_Driver/Src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    CAN_SOURCE_DIRS
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/dsdl_generated/src"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard/drivers/stm32"

    INCLUDES
        "${MCU_ROOT}/Inc"
        "${MCU_ROOT}/Drivers/STM32L4xx_HAL_Driver/Inc"
        "${MCU_ROOT}/Drivers/CMSIS/Include"
        "${MCU_ROOT}/Drivers/CMSIS/Device/ST/STM32L4xx/Include"

    CAN_INCLUDES
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard/drivers/stm32"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/dsdl_generated/include"

    DEFINES
        ${MCU_PART}
        L431                        
        USE_FULL_LL_DRIVER
        HSE_STARTUP_TIMEOUT=100
        LSE_STARTUP_TIMEOUT=5000
        HSI_VALUE=16000000
        LSI_VALUE=32000
        VDD_VALUE=3300
        DATA_CACHE_ENABLE=1
        INSTRUCTION_CACHE_ENABLE=0
        PREFETCH_ENABLE=1
)


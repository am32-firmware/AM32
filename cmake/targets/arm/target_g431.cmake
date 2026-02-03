set(MCU_ROOT "Mcu/g431")
set(MCU_PART "STM32G431xx")

am32_process_family(
    FAMILY            "G431"
    ARCH              "ARM"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/ldscript.ld"
    CAN_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/ldscript_CAN.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/STM32G431xx.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS         -mcpu=cortex-m4 -mthumb
    
    # PASS DIRECTORIES, NOT FILES
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup/gcc"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/STM32G4xx_HAL_Driver/Src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    CAN_SOURCE_DIRS
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/dsdl_generated/src"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard"
        "${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard/drivers/stm32"

    INCLUDES
        ${MCU_ROOT}/Inc
        ${MCU_ROOT}/Drivers/STM32G4xx_HAL_Driver/Inc
        ${MCU_ROOT}/Drivers/CMSIS/Include
        ${MCU_ROOT}/Drivers/CMSIS/Device/ST/STM32G4xx/Include

    CAN_INCLUDES
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN/libcanard/drivers/stm32
        ${CMAKE_SOURCE_DIR}/Src/DroneCAN/dsdl_generated/include

    DEFINES
        ${MCU_PART}
        G431                        
        USE_FULL_LL_DRIVER
        HSE_VALUE=8000000
        PREFETCH_ENABLE=1
)


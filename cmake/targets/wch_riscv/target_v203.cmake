set(MCU_ROOT "Mcu/v203")
set(MCU_PART "CH32V203")

am32_process_family(
    FAMILY            "V203"
    ARCH              "WCH_RISCV"
    LINKER_SCRIPT     "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Link.ld"
    SVD_FILE          "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/CH32V203xx.svd"
    OCD_CONFIG_FILE   "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/openocd.cfg"
    
    CPU_FLAGS
        -march=rv32imac 
        -mabi=ilp32 
        -msmall-data-limit=8 
        -msave-restore 
        -fmessage-length=0 
        -ffunction-sections 
        -fdata-sections 
        -fno-common 
        -nostartfiles
    
    # PASS DIRECTORIES, NOT FILES
    SOURCE_DIRS       
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Startup"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/Core"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/Peripheral/src"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Drivers/Debug"
        "${CMAKE_SOURCE_DIR}/${MCU_ROOT}/Src"

    INCLUDES
        ${MCU_ROOT}/Inc
        ${MCU_ROOT}/Drivers/Peripheral/inc
        ${MCU_ROOT}/Drivers/Core
        ${MCU_ROOT}/Drivers/Debug

    DEFINES     
        ${MCU_PART}
        V230
        MCU_FLASH_START=0x08000000
)

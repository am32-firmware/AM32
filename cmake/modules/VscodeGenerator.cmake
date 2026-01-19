function(am32_generate_launch_json)
    set(VSCODE_DIR "${CMAKE_SOURCE_DIR}/.vscode")
    file(MAKE_DIRECTORY "${VSCODE_DIR}")
    
    if(CMAKE_HOST_WIN32)
        set(EXT ".exe")
    else()
        set(EXT "")
    endif()

    # Tool Definitions
    set(ARM_GDB   "${CMAKE_SOURCE_DIR}/tools/gcc-arm/bin/arm-none-eabi-gdb${EXT}")
    set(ARM_OCD   "${CMAKE_SOURCE_DIR}/tools/openocd-arm/bin/openocd${EXT}")
    
    # Pointing to the WCH specific directories
    set(WCH_RISCV_GDB "${CMAKE_SOURCE_DIR}/tools/gcc-wch-riscv/bin/riscv-none-embed-gdb${EXT}")
    set(WCH_RISCV_OCD "${CMAKE_SOURCE_DIR}/tools/openocd-wch-riscv/bin/openocd${EXT}")

    set(CONFIGS "")

    macro(add_debug_entry NAME TARGET_CFG SVD_FILE IS_WCH_RISCV)
        if(${IS_WCH_RISCV})
            set(GDB_PATH "${WCH_RISCV_GDB}")
            set(OCD_PATH "${WCH_RISCV_OCD}")
        else()
            set(GDB_PATH "${ARM_GDB}")
            set(OCD_PATH "${ARM_OCD}")
        endif()

        string(APPEND CONFIGS "
        {
            \"name\": \"${NAME}\",
            \"type\": \"cortex-debug\",
            \"request\": \"launch\",
            \"servertype\": \"openocd\",
            \"cwd\": \"\${workspaceFolder}\",
            \"executable\": \"\${command:cmake.launchTargetPath}\",
            \"serverpath\": \"${OCD_PATH}\",
            \"gdbPath\": \"${GDB_PATH}\",
            \"configFiles\": [ \"\${workspaceFolder}/${TARGET_CFG}\" ],
            \"svdFile\": \"\${workspaceFolder}/${SVD_FILE}\",
            \"runToEntryPoint\": \"main\",
            \"showDevDebugOutput\": \"none\"
        },")
    endmacro()

    # --- Add The Families ---
    add_debug_entry("Debug GD32 E230"   "${CMAKE_SOURCE_DIR}/Mcu/e230/openocd.cfg" "Mcu/e230/GD32E230.svd"      FALSE)
    add_debug_entry("Debug STM32 F031"  "${CMAKE_SOURCE_DIR}/Mcu/f031/openocd.cfg" "Mcu/f031/STM32F0x1.svd"     FALSE)
    add_debug_entry("Debug STM32 F051"  "${CMAKE_SOURCE_DIR}/Mcu/f051/openocd.cfg" "Mcu/f051/STM32F0x1.svd"     FALSE)
    add_debug_entry("Debug AT32 F415"   "${CMAKE_SOURCE_DIR}/Mcu/f415/openocd.cfg" "Mcu/f415/AT32F415xx_v2.svd" FALSE)
    add_debug_entry("Debug AT32 F421"   "${CMAKE_SOURCE_DIR}/Mcu/f421/openocd.cfg" "Mcu/f421/AT32F421xx_v2.svd" FALSE)
    add_debug_entry("Debug STM32 G031"  "${CMAKE_SOURCE_DIR}/Mcu/g031/openocd.cfg" "Mcu/g031/STM32F0x1.svd"     FALSE)
    add_debug_entry("Debug STM32 G071"  "${CMAKE_SOURCE_DIR}/Mcu/g071/openocd.cfg" "Mcu/g031/STM32G071.svd"     FALSE)
    add_debug_entry("Debug STM32 G431"  "${CMAKE_SOURCE_DIR}/Mcu/g431/openocd.cfg" "Mcu/g431/STM32G431xx.svd"   FALSE)
    add_debug_entry("Debug STM32 L431"  "${CMAKE_SOURCE_DIR}/Mcu/l431/openocd.cfg" "Mcu/l431/STM32L4x1.svd"     FALSE)

    add_debug_entry("Debug CH32V203 (WCH RISC-V)"  "${CMAKE_SOURCE_DIR}/Mcu/v203/openocd.cfg" "Mcu/v203/CH32V203xx.svd" TRUE)

    string(REGEX REPLACE ",$" "" CONFIGS "${CONFIGS}")

    set(FINAL_JSON "{
    \"version\": \"0.2.0\",
    \"configurations\": [${CONFIGS}
    ]
}")

    file(WRITE "${VSCODE_DIR}/launch.json" "${FINAL_JSON}")
    message(STATUS "[VSCode] Generated .vscode/launch.json")
endfunction()
IF(NOT DEFINED FPU)  
    SET(FPU "-mfloat-abi=hard -mfpu=fpv5-sp-d16")  
ENDIF()  

IF(NOT DEFINED SPECS)  
    SET(SPECS "--specs=nosys.specs")  
ENDIF()  

IF(NOT DEFINED DEBUG_CONSOLE_CONFIG)  
    SET(DEBUG_CONSOLE_CONFIG "-DSDK_DEBUGCONSOLE_UART")  
ENDIF()  

SET(CMAKE_ASM_FLAGS_DEBUG " \
    ${CMAKE_ASM_FLAGS_DEBUG} \
    -D__STARTUP_CLEAR_BSS \
    -DMCUXPRESSO_SDK \
    -DHAL_UART_ADAPTER_LOWPOWER=1 \
    -DCPU_MCXW716CMFTA \
    -DOSA_USED \
    -g \
    -mthumb \
    -mcpu=cortex-m33 \
    ${FPU} \
")
SET(CMAKE_ASM_FLAGS_RELEASE " \
    ${CMAKE_ASM_FLAGS_RELEASE} \
    -D__STARTUP_CLEAR_BSS \
    -DMCUXPRESSO_SDK \
    -DHAL_UART_ADAPTER_LOWPOWER=1 \
    -DCPU_MCXW716CMFTA \
    -DOSA_USED \
    -mthumb \
    -mcpu=cortex-m33 \
    ${FPU} \
")
SET(CMAKE_C_FLAGS_DEBUG " \
    ${CMAKE_C_FLAGS_DEBUG} \
    -include ${ProjDirPath}/source/app_preinclude.h \
    -include ${ProjDirPath}/source/mcux_config.h \
    -DDEBUG \
    -DSERIAL_USE_CONFIGURE_STRUCTURE=1 \
    -DgButtonSupported_d=1 \
    -DOSA_USED \
    -DSDK_COMPONENT_INTEGRATION=1 \
    -DFSL_OSA_TASK_ENABLE=1 \
    -DCR_INTEGER_PRINTF \
    -DMCXW716C \
    -DCPU_MCXW716CMFTA \
    -DCFG_BLE_PRJ=1 \
    -DENABLE_RAM_VECTOR_TABLE=1 \
    -DNXP_SSSAPI \
    -DNXP_ELE200 \
    -DHAL_FLASH_ROMAPI_DRIVER=1 \
    -DgUseHciTransportDownward_d=1 \
    -DTM_ENABLE_TIME_STAMP=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DHAL_UART_ADAPTER_LOWPOWER=1 \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DSERIAL_PORT_TYPE_RPMSG=1 \
    -DTIMER_PORT_TYPE_LPTMR=1 \
    -DUSE_RTOS=1 \
    -DGENERIC_LIST_LIGHT=1 \
    -DMULTICORE_APP=1 \
    -DgRngUseSecureSubSystem_d=1 \
    -DRPMSG_ADAPTER_NON_BLOCKING_MODE=1 \
    -DMBEDTLS_MCUX_ELS_PKC_API \
    -DgNvmSaveOnIdlePolicy_d=0 \
    -DSDK_OS_FREE_RTOS \
    -Og \
    -g \
    --specs=nano.specs \
    -Wall \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -std=gnu99 \
    -mcpu=cortex-m33 \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
")
SET(CMAKE_C_FLAGS_RELEASE " \
    ${CMAKE_C_FLAGS_RELEASE} \
    -include ${ProjDirPath}/source/app_preinclude.h \
    -include ${ProjDirPath}/source/mcux_config.h \
    -DNDEBUG \
    -DSERIAL_USE_CONFIGURE_STRUCTURE=1 \
    -DgButtonSupported_d=1 \
    -DOSA_USED \
    -DSDK_COMPONENT_INTEGRATION=1 \
    -DFSL_OSA_TASK_ENABLE=1 \
    -DCR_INTEGER_PRINTF \
    -DMCXW716C \
    -DCPU_MCXW716CMFTA \
    -DCFG_BLE_PRJ=1 \
    -DENABLE_RAM_VECTOR_TABLE=1 \
    -DNXP_SSSAPI \
    -DNXP_ELE200 \
    -DHAL_FLASH_ROMAPI_DRIVER=1 \
    -DgUseHciTransportDownward_d=1 \
    -DTM_ENABLE_TIME_STAMP=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DHAL_UART_ADAPTER_LOWPOWER=1 \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DSERIAL_PORT_TYPE_RPMSG=1 \
    -DTIMER_PORT_TYPE_LPTMR=1 \
    -DUSE_RTOS=1 \
    -DGENERIC_LIST_LIGHT=1 \
    -DMULTICORE_APP=1 \
    -DgRngUseSecureSubSystem_d=1 \
    -DRPMSG_ADAPTER_NON_BLOCKING_MODE=1 \
    -DMBEDTLS_MCUX_ELS_PKC_API \
    -DgNvmSaveOnIdlePolicy_d=0 \
    -DSDK_OS_FREE_RTOS \
    -Os \
    --specs=nano.specs \
    -Wall \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -std=gnu99 \
    -mcpu=cortex-m33 \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
")
SET(CMAKE_CXX_FLAGS_DEBUG " \
    ${CMAKE_CXX_FLAGS_DEBUG} \
    -DDEBUG \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DHAL_UART_ADAPTER_LOWPOWER=1 \
    -DCPU_MCXW716CMFTA \
    -DOSA_USED \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DSERIAL_PORT_TYPE_RPMSG=1 \
    -DTIMER_PORT_TYPE_LPTMR=1 \
    -DUSE_RTOS=1 \
    -DgRngUseSecureSubSystem_d=1 \
    -DRPMSG_ADAPTER_NON_BLOCKING_MODE=1 \
    -DMBEDTLS_MCUX_ELS_PKC_API \
    -DgNvmSaveOnIdlePolicy_d=0 \
    -DSDK_OS_FREE_RTOS \
    -Og \
    -g \
    --specs=nano.specs \
    -Wall \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -fno-rtti \
    -fno-exceptions \
    -mcpu=cortex-m33 \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
")
SET(CMAKE_CXX_FLAGS_RELEASE " \
    ${CMAKE_CXX_FLAGS_RELEASE} \
    -DNDEBUG \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DHAL_UART_ADAPTER_LOWPOWER=1 \
    -DCPU_MCXW716CMFTA \
    -DOSA_USED \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DSERIAL_PORT_TYPE_RPMSG=1 \
    -DTIMER_PORT_TYPE_LPTMR=1 \
    -DUSE_RTOS=1 \
    -DgRngUseSecureSubSystem_d=1 \
    -DRPMSG_ADAPTER_NON_BLOCKING_MODE=1 \
    -DMBEDTLS_MCUX_ELS_PKC_API \
    -DgNvmSaveOnIdlePolicy_d=0 \
    -DSDK_OS_FREE_RTOS \
    -Os \
    --specs=nano.specs \
    -Wall \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -fno-rtti \
    -fno-exceptions \
    -mcpu=cortex-m33 \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG " \
    ${CMAKE_EXE_LINKER_FLAGS_DEBUG} \
    -g \
    -Xlinker \
    --defsym=gUseNVMLink_d=1 \
    -Xlinker \
    --defsym=gEraseNVMLink_d=1 \
    -Xlinker \
    --defsym=__ram_vector_table__=1 \
    -Xlinker \
    --defsym=gFlashNbuImage_d=1 \
    -Xlinker \
    --defsym=gUseProdInfoLegacyMode_d=1 \
    -Xlinker \
    --defsym=__use_shmem__=1 \
    -Xlinker \
    -Map=output.map \
    -Wall \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -Wl,--gc-sections \
    -Wl,-static \
    -Wl,--print-memory-usage \
    -mcpu=cortex-m33 \
    ${FPU} \
    ${SPECS} \
    -T\"${ProjDirPath}/MCXW716C/armgcc/connectivity_ble.ld\" -static \
")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE " \
    ${CMAKE_EXE_LINKER_FLAGS_RELEASE} \
    -Xlinker \
    --defsym=gUseNVMLink_d=1 \
    -Xlinker \
    --defsym=gEraseNVMLink_d=1 \
    -Xlinker \
    --defsym=__ram_vector_table__=1 \
    -Xlinker \
    --defsym=gFlashNbuImage_d=1 \
    -Xlinker \
    --defsym=gUseProdInfoLegacyMode_d=1 \
    -Xlinker \
    --defsym=__use_shmem__=1 \
    -Xlinker \
    -Map=output.map \
    -Wall \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -Wl,--gc-sections \
    -Wl,-static \
    -Wl,--print-memory-usage \
    -mcpu=cortex-m33 \
    ${FPU} \
    ${SPECS} \
    -T\"${ProjDirPath}/MCXW716C/armgcc/connectivity_ble.ld\" -static \
")

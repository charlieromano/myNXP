# Add set(CONFIG_USE_middleware_wireless_ble_host_component_lib true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

if(CONFIG_TOOLCHAIN STREQUAL iar AND CONFIG_FPU STREQUAL SP_FPU)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE
-Wl,--start-group
${CMAKE_CURRENT_LIST_DIR}/../../../bluetooth/host/lib/lib_ble_host_cm33_iar.a
-Wl,--end-group
)
endif()

if((CONFIG_TOOLCHAIN STREQUAL mcux OR CONFIG_TOOLCHAIN STREQUAL armgcc) AND CONFIG_FPU STREQUAL SP_FPU)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE
-Wl,--start-group
${CMAKE_CURRENT_LIST_DIR}/../../../bluetooth/host/lib/lib_ble_host_cm33_gcc.a
-Wl,--end-group
)
endif()

if(CONFIG_TOOLCHAIN STREQUAL iar AND CONFIG_FPU STREQUAL NO_FPU AND CONFIG_DSP STREQUAL NO_DSP)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE
-Wl,--start-group
${CMAKE_CURRENT_LIST_DIR}/../../../bluetooth/host/lib/lib_ble_host_cm33_nofp_nodsp_iar.a
-Wl,--end-group
)
endif()

if(CONFIG_TOOLCHAIN STREQUAL armgcc AND CONFIG_FPU STREQUAL NO_FPU AND CONFIG_DSP STREQUAL NO_DSP)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE
-Wl,--start-group
${CMAKE_CURRENT_LIST_DIR}/../../../bluetooth/host/lib/lib_ble_host_cm33_nofp_nodsp_gcc.a
-Wl,--end-group
)
endif()



# Add set(CONFIG_USE_middleware_wireless_ble_host true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
${CMAKE_CURRENT_LIST_DIR}/port/fwk_generic_list.c
${CMAKE_CURRENT_LIST_DIR}/port/fwk_messaging.c
${CMAKE_CURRENT_LIST_DIR}/port/fwk_os_abs.c
${CMAKE_CURRENT_LIST_DIR}/port/fwk_timer_manager.c
${CMAKE_CURRENT_LIST_DIR}/host/config/ble_globals.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_LIST_DIR}/host/interface
${CMAKE_CURRENT_LIST_DIR}/host/config
${CMAKE_CURRENT_LIST_DIR}/port
)



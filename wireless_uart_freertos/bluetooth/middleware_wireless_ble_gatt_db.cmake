# Add set(CONFIG_USE_middleware_wireless_ble_gatt_db true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
${CMAKE_CURRENT_LIST_DIR}/application/common/gatt_db/gatt_database.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_LIST_DIR}/application/common/gatt_db/macros
${CMAKE_CURRENT_LIST_DIR}/application/common/gatt_db
${CMAKE_CURRENT_LIST_DIR}/application/common
)



# CROSS COMPILER SETTING
set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 3.10.0)

# THE VERSION NUMBER
SET (MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION 2)
SET (MCUXPRESSO_CMAKE_FORMAT_MINOR_VERSION 0)

include(ide_overrides.cmake OPTIONAL)

if(CMAKE_SCRIPT_MODE_FILE)
message("${MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION}")
return()
endif()


set(CMAKE_STATIC_LIBRARY_PREFIX)
set(CMAKE_STATIC_LIBRARY_SUFFIX)

set(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
set(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
set(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
set(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(wireless_uart_freertos)

enable_language(ASM)

set(MCUX_BUILD_TYPES debug release)

set(MCUX_SDK_PROJECT_NAME wireless_uart_freertos.elf)

if (NOT DEFINED SdkRootDirPath)
SET(SdkRootDirPath ${ProjDirPath}/)
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME}
"${ProjDirPath}/board/pin_mux.c"
"${ProjDirPath}/board/pin_mux.h"
"${ProjDirPath}/source/FreeRTOSConfig.h"
"${ProjDirPath}/source/FreeRTOSConfig_Gen.h"
"${ProjDirPath}/source/app.h"
"${ProjDirPath}/source/app_advertiser.c"
"${ProjDirPath}/source/app_advertiser.h"
"${ProjDirPath}/source/app_ble_init.c"
"${ProjDirPath}/source/app_config.c"
"${ProjDirPath}/source/app_conn.c"
"${ProjDirPath}/source/app_conn.h"
"${ProjDirPath}/source/app_connection.c"
"${ProjDirPath}/source/app_low_power.c"
"${ProjDirPath}/source/app_nvm.c"
"${ProjDirPath}/source/app_preinclude.h"
"${ProjDirPath}/source/app_preinclude_common.h"
"${ProjDirPath}/source/app_scanner.c"
"${ProjDirPath}/source/app_scanner.h"
"${ProjDirPath}/source/app_services_init.c"
"${ProjDirPath}/source/ble_init.c"
"${ProjDirPath}/source/ble_init.h"
"${ProjDirPath}/source/board.c"
"${ProjDirPath}/source/board.h"
"${ProjDirPath}/source/board_comp.c"
"${ProjDirPath}/source/board_comp.h"
"${ProjDirPath}/source/board_dcdc.c"
"${ProjDirPath}/source/board_dcdc.h"
"${ProjDirPath}/source/board_lp.c"
"${ProjDirPath}/source/board_lp.h"
"${ProjDirPath}/source/board_platform.h"
"${ProjDirPath}/source/clock_config.c"
"${ProjDirPath}/source/clock_config.h"
"${ProjDirPath}/source/gatt_db.h"
"${ProjDirPath}/source/gatt_uuid128.h"
"${ProjDirPath}/source/hardware_init.c"
"${ProjDirPath}/source/main.c"
"${ProjDirPath}/source/mcux_config.h"
"${ProjDirPath}/source/pin_mux_lp.h"
"${ProjDirPath}/source/wireless_uart.c"
"${ProjDirPath}/source/wireless_uart.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
${ProjDirPath}/source
${ProjDirPath}/board
)

set_source_files_properties("${ProjDirPath}/source/FreeRTOSConfig.h" PROPERTIES COMPONENT_CONFIG_FILE "middleware_freertos-kernel_config")
set_source_files_properties("${ProjDirPath}/source/FreeRTOSConfig_Gen.h" PROPERTIES COMPONENT_CONFIG_FILE "middleware_freertos-kernel_config")

if(CMAKE_BUILD_TYPE STREQUAL debug)
target_compile_definitions(${MCUX_SDK_PROJECT_NAME}  PRIVATE $<$<COMPILE_LANGUAGE:C>:SSS_CONFIG_FILE="fsl_sss_config_elemu.h" SSCP_CONFIG_FILE="fsl_sscp_config_elemu.h">)
endif()

if(CMAKE_BUILD_TYPE STREQUAL release)
target_compile_definitions(${MCUX_SDK_PROJECT_NAME}  PRIVATE $<$<COMPILE_LANGUAGE:C>:SSS_CONFIG_FILE="fsl_sss_config_elemu.h" SSCP_CONFIG_FILE="fsl_sscp_config_elemu.h">)
endif()

include(${ProjDirPath}/all_lib_device.cmake)

IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)
SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")
ENDIF()

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Osrec ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/wireless_uart_frdmmcxw71.srec)

set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES "output.map;${EXECUTABLE_OUTPUT_PATH}/wireless_uart_frdmmcxw71.srec")

# wrap all libraries with -Wl,--start-group -Wl,--end-group to prevent link order issue
group_link_libraries()


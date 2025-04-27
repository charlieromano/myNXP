# Add set(CONFIG_USE_middleware_secure-subsystem_elemu true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
${CMAKE_CURRENT_LIST_DIR}/src/sscp/fsl_sss_mgmt.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_LIST_DIR}/inc
${CMAKE_CURRENT_LIST_DIR}/inc/elemu
${CMAKE_CURRENT_LIST_DIR}/src/sscp
)

if(CONFIG_USE_COMPONENT_CONFIGURATION)
message("===>Import configuration from ${CMAKE_CURRENT_LIST_FILE}")

target_compile_definitions(${MCUX_SDK_PROJECT_NAME} PUBLIC
-DSSS_CONFIG_FILE="fsl_sss_config_elemu.h"
-DSSCP_CONFIG_FILE="fsl_sscp_config_elemu.h"
)


endif()


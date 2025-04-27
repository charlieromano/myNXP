# Add set(CONFIG_USE_middleware_secure-subsystem_elemu_port_kw45_k4w1 true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_aes.c
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_aes_cmac.c
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_ccm.c
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_ecdh.c
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_hmac_sha256.c
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_init.c
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1/sss_sha256.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_LIST_DIR}/port
${CMAKE_CURRENT_LIST_DIR}/port/kw45_k4w1
)



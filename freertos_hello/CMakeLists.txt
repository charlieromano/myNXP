# CROSS COMPILER SETTING
set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 3.10.0)

# THE VERSION NUMBER
SET(MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION 2)
SET(MCUXPRESSO_CMAKE_FORMAT_MINOR_VERSION 0)

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

project(freertos_hello)

enable_language(ASM)

set(MCUX_BUILD_TYPES debug release)
set(MCUX_SDK_PROJECT_NAME freertos_hello.elf)

if (NOT DEFINED SdkRootDirPath)
SET(SdkRootDirPath ${ProjDirPath}/)
endif()

include(${ProjDirPath}/flags.cmake)
include(${ProjDirPath}/config.cmake)

# Automatically collect user .c/.h files
file(GLOB_RECURSE USER_SRC "${ProjDirPath}/src/*.c")
file(GLOB_RECURSE USER_INC "${ProjDirPath}/inc/*.h")

add_executable(${MCUX_SDK_PROJECT_NAME}
    "${ProjDirPath}/board/app.h"
    "${ProjDirPath}/board/board.c"
    "${ProjDirPath}/board/board.h"
    "${ProjDirPath}/board/clock_config.c"
    "${ProjDirPath}/board/clock_config.h"
    "${ProjDirPath}/board/hardware_init.c"
    "${ProjDirPath}/board/pin_mux.c"
    "${ProjDirPath}/board/pin_mux.h"
    "${ProjDirPath}/source/FreeRTOSConfig_Gen.h"
    "${ProjDirPath}/source/mcux_config.h"
    ${USER_SRC}
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/source
    ${ProjDirPath}/board
    ${ProjDirPath}/inc
    ${ProjDirPath}/src
)

set_source_files_properties("${ProjDirPath}/source/FreeRTOSConfig_Gen.h" PROPERTIES COMPONENT_CONFIG_FILE "middleware_freertos-kernel_config")

include(${ProjDirPath}/all_lib_device.cmake)

if(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)
    set(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")
endif()

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

add_custom_command(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
    -Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/freertos_hello.bin
)

set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES
    "output.map;${EXECUTABLE_OUTPUT_PATH}/freertos_hello.bin"
)

group_link_libraries()

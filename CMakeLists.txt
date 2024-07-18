# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
   set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(PICO_SDK_PATH ${USERHOME}/.pico-sdk/sdk/1.5.1)
set(PICO_TOOLCHAIN_PATH ${USERHOME}/.pico-sdk/toolchain/13_2_Rel1)
if(WIN32)
    set(pico-sdk-tools_DIR ${USERHOME}/.pico-sdk/tools/1.5.1)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config.cmake)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config-version.cmake)
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(pico_w_CO2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(pico_w_CO2 main.c )

pico_set_program_name(pico_w_CO2 "pico_w_CO2")
pico_set_program_version(pico_w_CO2 "0.4")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_w_CO2 1)
pico_enable_stdio_usb(pico_w_CO2 1)

# Add the standard library to the build
target_link_libraries(pico_w_CO2
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_w_CO2 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/wifi
)

set(PICO_DEOPTIMIZED_DEBUG 1 CACHE BOOL "Deoptimized debug")

add_subdirectory(pico-ds3231/lib)
add_subdirectory(gfx_pack)
add_subdirectory(ee895)
add_subdirectory(cdm7162)
add_subdirectory(common)
add_subdirectory(sunrise)
add_subdirectory(sunlight)
add_subdirectory(scd30)
add_subdirectory(scd41)
add_subdirectory(cozir-lp3)
add_subdirectory(cm1107n)
add_subdirectory(mux)
add_subdirectory(soap)
add_subdirectory(wifi)
add_subdirectory(sensors)

target_sources(pico_w_CO2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/sensor_config.h)
target_sources(pico_w_CO2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/config_map.h)
target_sources(pico_w_CO2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/common_include.h)

# Add any user requested libraries
target_link_libraries(pico_w_CO2 
        hardware_i2c
        pico-ds3231
        pico-gfx_pack
        pico-ee895
        pico-cdm7162
        pico-sunrise
        pico-sunlight
        hardware_exception
        pico-scd30
        pico-cozir-lp3
        pico-cm1107n
        pico-mux
        pico-soap
        pico_multicore
        pico-wifi
        pico-sensors
        )

pico_add_extra_outputs(pico_w_CO2)


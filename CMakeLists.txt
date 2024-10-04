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
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
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

target_compile_definitions(pico_w_CO2 PRIVATE
  PICO_STACK_SIZE=4096
)

# Add the standard include files to the build
target_include_directories(pico_w_CO2 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/wifi
)

set(PICO_DEOPTIMIZED_DEBUG 0 CACHE BOOL "Deoptimized debug")
set(FULL_BUILD 1)
add_compile_definitions(-DFULL_BUILD=${FULL_BUILD})

add_subdirectory(rtc)
add_subdirectory(gfx_pack)
add_subdirectory(sensors)
add_subdirectory(common)
add_subdirectory(eeprom)
add_subdirectory(config)
add_subdirectory(display)
add_subdirectory(error_handler)

if(FULL_BUILD)
add_subdirectory(soap)
add_subdirectory(wifi)
add_subdirectory(cobs)
add_subdirectory(uart)
add_subdirectory(service_comm)
endif()

target_compile_options(pico_w_CO2 PRIVATE "-g")

target_sources(pico_w_CO2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/sensor_config.h)
target_sources(pico_w_CO2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/config_map.h)
target_sources(pico_w_CO2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/error_codes.h)



# Add any user requested libraries
target_link_libraries(pico_w_CO2 
        pico_multicore
        pico-sensors
        pico-config
        pico-display
        pico-error_handler
        )

if (FULL_BUILD)
  target_link_libraries(pico_w_CO2
  pico-soap
  pico-service_comm
  pico-wifi
  )
endif()
        
pico_add_extra_outputs(pico_w_CO2)
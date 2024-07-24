add_library(pico-power STATIC)

target_include_directories(pico-power
  INTERFACE
)

target_link_libraries(pico-power
PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-power PRIVATE "-g")

target_sources(pico-power
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/power.c
  ${CMAKE_CURRENT_LIST_DIR}/power.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/common_include.h
)
add_library(pico-power INTERFACE)

target_link_libraries(pico-power
INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-power
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/power.c
  ${CMAKE_CURRENT_LIST_DIR}/power.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/common_include.h
)
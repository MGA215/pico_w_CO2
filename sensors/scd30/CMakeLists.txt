add_library(pico-scd30 INTERFACE)

target_link_libraries(pico-scd30
INTERFACE
  hardware_i2c
  pico-common
  pico-power
)

target_sources(pico-scd30
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/scd30.c
  ${CMAKE_CURRENT_LIST_DIR}/scd30.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
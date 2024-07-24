add_library(pico-scd41 INTERFACE)

target_link_libraries(pico-scd41
INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-scd30
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/scd41.c
  ${CMAKE_CURRENT_LIST_DIR}/scd41.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
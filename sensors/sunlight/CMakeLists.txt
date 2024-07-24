add_library(pico-sunlight INTERFACE)

target_link_libraries(pico-sunlight
INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-sunlight
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/sunlight.c
  ${CMAKE_CURRENT_LIST_DIR}/sunlight.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
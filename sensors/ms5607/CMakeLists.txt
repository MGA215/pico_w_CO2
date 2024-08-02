add_library(pico-ms5607 INTERFACE)

target_link_libraries(pico-ms5607
INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-ms5607
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/ms5607.c
  ${CMAKE_CURRENT_LIST_DIR}/ms5607.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
)
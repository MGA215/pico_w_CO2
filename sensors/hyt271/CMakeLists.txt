add_library(pico-hyt271 INTERFACE)

target_link_libraries(pico-hyt271
INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-hyt271
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hyt271.c
  ${CMAKE_CURRENT_LIST_DIR}/hyt271.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
)
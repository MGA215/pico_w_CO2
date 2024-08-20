add_library(pico-cdm7162 INTERFACE)

target_link_libraries(pico-cdm7162
INTERFACE
  hardware_i2c
  pico-common
  pico-power
)

target_sources(pico-cdm7162
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/cdm7162.c
  ${CMAKE_CURRENT_LIST_DIR}/cdm7162.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
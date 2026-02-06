add_library(pico-generic-co2 INTERFACE)

target_link_libraries(pico-generic-co2
INTERFACE
  hardware_i2c
  pico-common
  pico-power
)

target_sources(pico-generic-co2
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/generic_co2.c
  ${CMAKE_CURRENT_LIST_DIR}/generic_co2.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
add_library(pico-sunrise INTERFACE)

target_link_libraries(pico-sunrise
INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-sunrise
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/sunrise.c
  ${CMAKE_CURRENT_LIST_DIR}/sunrise.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
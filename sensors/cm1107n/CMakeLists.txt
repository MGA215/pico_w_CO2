add_library(pico-cm1107n INTERFACE)

target_link_libraries(pico-cm1107n
INTERFACE
  hardware_i2c
  pico-common
  pico-power
)

target_sources(pico-cm1107n
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/cm1107n.c
  ${CMAKE_CURRENT_LIST_DIR}/cm1107n.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
add_library(pico-ee872 INTERFACE)

target_link_libraries(pico-ee872
INTERFACE
  pico-common
  pico-power
)

target_sources(pico-ee872
INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/ee872.c
  ${CMAKE_CURRENT_LIST_DIR}/ee872.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
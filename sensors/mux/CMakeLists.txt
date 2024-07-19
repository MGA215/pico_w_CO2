add_library(pico-mux INTERFACE)

target_include_directories(pico-mux
  INTERFACE
)

target_link_libraries(pico-mux
  INTERFACE
  hardware_i2c
  pico-common
)

target_sources(pico-mux
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/mux.c
  ${CMAKE_CURRENT_LIST_DIR}/mux.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
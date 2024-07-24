add_library(pico-mux STATIC)

target_include_directories(pico-mux
  INTERFACE
)

target_link_libraries(pico-mux
  PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-mux PRIVATE "-g")

target_sources(pico-mux
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/mux.c
  ${CMAKE_CURRENT_LIST_DIR}/mux.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
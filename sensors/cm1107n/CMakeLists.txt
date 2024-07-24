add_library(pico-cm1107n STATIC)

target_include_directories(pico-cm1107n
  INTERFACE
)

target_link_libraries(pico-cm1107n
  PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-cm1107n PRIVATE "-g")

target_sources(pico-cm1107n
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/cm1107n.c
  ${CMAKE_CURRENT_LIST_DIR}/cm1107n.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
add_library(pico-scd30 STATIC)

target_include_directories(pico-scd30
  INTERFACE
)

target_link_libraries(pico-scd30
PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-scd30 PRIVATE "-g")

target_sources(pico-scd30
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/scd30.c
  ${CMAKE_CURRENT_LIST_DIR}/scd30.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
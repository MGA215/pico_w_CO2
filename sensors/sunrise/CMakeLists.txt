add_library(pico-sunrise STATIC)

target_include_directories(pico-sunrise
  INTERFACE
)

target_link_libraries(pico-sunrise
PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-sunrise PRIVATE "-g")

target_sources(pico-sunrise
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/sunrise.c
  ${CMAKE_CURRENT_LIST_DIR}/sunrise.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
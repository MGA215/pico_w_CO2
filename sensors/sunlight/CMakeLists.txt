add_library(pico-sunlight STATIC)

target_include_directories(pico-sunlight
  INTERFACE
)

target_link_libraries(pico-sunlight
PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-sunlight PRIVATE "-g")

target_sources(pico-sunlight
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/sunlight.c
  ${CMAKE_CURRENT_LIST_DIR}/sunlight.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
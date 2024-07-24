add_library(pico-ee895 STATIC)

target_include_directories(pico-ee895
  INTERFACE
)

target_link_libraries(pico-ee895
PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-ee895 PRIVATE "-g")

target_sources(pico-ee895
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/ee895.c
  ${CMAKE_CURRENT_LIST_DIR}/ee895.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
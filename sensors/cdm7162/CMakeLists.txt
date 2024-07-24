add_library(pico-cdm7162 STATIC)

target_include_directories(pico-cdm7162
  INTERFACE
)

target_link_libraries(pico-cdm7162
  PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-cdm7162 PRIVATE "-g")

target_sources(pico-cdm7162
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/cdm7162.c
  ${CMAKE_CURRENT_LIST_DIR}/cdm7162.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
add_library(pico-cozir-lp3 STATIC)

target_include_directories(pico-cozir-lp3
  INTERFACE
)

target_link_libraries(pico-cozir-lp3
  PRIVATE
  hardware_i2c
  pico-common
)

target_compile_options(pico-cozir-lp3 PRIVATE "-g")

target_sources(pico-cozir-lp3
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/cozir-lp3.c
  ${CMAKE_CURRENT_LIST_DIR}/cozir-lp3.h
  ${CMAKE_CURRENT_LIST_DIR}/../../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/../../common/functions.h
)
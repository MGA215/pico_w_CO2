add_library(pico-common STATIC)

target_include_directories(pico-common
  PRIVATE
)

target_compile_options(pico-common PRIVATE "-g")

target_sources(pico-common
PRIVATE
${CMAKE_CURRENT_LIST_DIR}/functions.c
${CMAKE_CURRENT_LIST_DIR}/serialize.c
${CMAKE_CURRENT_LIST_DIR}/debug.c
${CMAKE_CURRENT_LIST_DIR}/i2c_extras.c
)

target_sources(pico-common INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/functions.h
  ${CMAKE_CURRENT_LIST_DIR}/constants.h
  ${CMAKE_CURRENT_LIST_DIR}/structs.h
  ${CMAKE_CURRENT_LIST_DIR}/serialize.h
  ${CMAKE_CURRENT_LIST_DIR}/debug.h
  ${CMAKE_CURRENT_LIST_DIR}/i2c_extras.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)

target_sources(pico-common INTERFACE ${CMAKE_CURRENT_LIST_DIR}/common_include.h)


target_link_libraries(pico-common
  PUBLIC
  pico_stdlib
  hardware_i2c
)
add_library(pico-common INTERFACE)


target_sources(pico-common
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/functions.c
  ${CMAKE_CURRENT_LIST_DIR}/serialize.c
  ${CMAKE_CURRENT_LIST_DIR}/debug.c
  ${CMAKE_CURRENT_LIST_DIR}/i2c_extras.c
  ${CMAKE_CURRENT_LIST_DIR}/functions.h
  ${CMAKE_CURRENT_LIST_DIR}/constants.h
  ${CMAKE_CURRENT_LIST_DIR}/structs.h
  ${CMAKE_CURRENT_LIST_DIR}/serialize.h
  ${CMAKE_CURRENT_LIST_DIR}/debug.h
  ${CMAKE_CURRENT_LIST_DIR}/i2c_extras.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/shared.c
  ${CMAKE_CURRENT_LIST_DIR}/shared.h
)

target_sources(pico-common INTERFACE ${CMAKE_CURRENT_LIST_DIR}/common_include.h)

target_link_libraries(pico-common
  INTERFACE
  pico_stdlib
  hardware_i2c
  pico-service_comm
  pico-ds3231
)
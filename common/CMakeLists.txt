add_library(pico-common INTERFACE)

target_include_directories(pico-common
  INTERFACE
)

target_link_libraries(pico-common
  INTERFACE
)

target_sources(pico-common
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/functions.c
  ${CMAKE_CURRENT_LIST_DIR}/functions.h
  ${CMAKE_CURRENT_LIST_DIR}/constants.h
  ${CMAKE_CURRENT_LIST_DIR}/structs.h
  ${CMAKE_CURRENT_LIST_DIR}/serialize.c
  ${CMAKE_CURRENT_LIST_DIR}/serialize.h
  ${CMAKE_CURRENT_LIST_DIR}/debug.c
  ${CMAKE_CURRENT_LIST_DIR}/debug.h
)
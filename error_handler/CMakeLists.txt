add_library(pico-error_handler INTERFACE)

target_include_directories(pico-error_handler
  INTERFACE
)

target_link_libraries(pico-error_handler
  INTERFACE
  pico-common
  hardware_exception
)

target_sources(pico-error_handler
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/error_handler.c
  ${CMAKE_CURRENT_LIST_DIR}/error_handler.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)
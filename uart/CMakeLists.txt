add_library(pico-uart INTERFACE)

target_include_directories(pico-uart
  INTERFACE
)

target_link_libraries(pico-uart
  INTERFACE
  pico-common
  pico-cobs
)

target_sources(pico-uart
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/uart.c
  ${CMAKE_CURRENT_LIST_DIR}/uart.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)
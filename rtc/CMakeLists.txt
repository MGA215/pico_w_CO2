add_library(pico-rtc INTERFACE)

target_include_directories(pico-rtc
  INTERFACE
)

add_subdirectory(pico-ds3231/lib)

target_link_libraries(pico-rtc
  INTERFACE
  pico-common
  pico-ds3231
)

target_sources(pico-rtc
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/rtc.c
  ${CMAKE_CURRENT_LIST_DIR}/rtc.h
  ${CMAKE_CURRENT_LIST_DIR}/timeconst.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)
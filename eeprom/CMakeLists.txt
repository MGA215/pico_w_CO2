add_library(pico-eeprom INTERFACE)

target_include_directories(pico-eeprom
  INTERFACE
)

target_link_libraries(pico-eeprom
  INTERFACE
  pico-common
  hardware_i2c
)

target_sources(pico-eeprom
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/eeprom.c
  ${CMAKE_CURRENT_LIST_DIR}/eeprom.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)
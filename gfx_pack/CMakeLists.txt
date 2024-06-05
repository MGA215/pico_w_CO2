add_library(pico-gfx_pack INTERFACE)

target_include_directories(pico-gfx_pack
  INTERFACE
)

target_link_libraries(pico-gfx_pack
  INTERFACE
  hardware_i2c
  hardware_spi
  hardware_pwm
)

target_sources(pico-gfx_pack
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/gfx_pack.c
  ${CMAKE_CURRENT_LIST_DIR}/gfx_pack.h
  ${CMAKE_CURRENT_LIST_DIR}/monospace_font.c
)
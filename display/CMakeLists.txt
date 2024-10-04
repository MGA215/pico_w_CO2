add_library(pico-display INTERFACE)

target_include_directories(pico-display
  INTERFACE
)

target_link_libraries(pico-display
  INTERFACE
  pico-common
  pico-gfx_pack
  pico-rtc
  )
  
if (FULL_BUILD)
  target_link_libraries(pico-display INTERFACE
    pico-wifi
    pico-soap
    )
endif()
  

target_sources(pico-display
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/display.c
  ${CMAKE_CURRENT_LIST_DIR}/display.h
)
add_library(pico-sensors INTERFACE)

target_include_directories(pico-sensors
  INTERFACE
)

add_subdirectory(ee895)
add_subdirectory(cdm7162)
add_subdirectory(sunrise)
add_subdirectory(sunlight)
add_subdirectory(scd30)
add_subdirectory(scd41)
add_subdirectory(cozir-lp3)
add_subdirectory(cm1107n)
add_subdirectory(mux)

target_link_libraries(pico-sensors
  INTERFACE
  pico-common
  pico-ee895
  pico-cdm7162
  pico-sunrise
  pico-sunlight
  pico-scd30
  pico-scd41
  pico-cozir-lp3
  pico-cm1107n
  pico-mux
)

target_sources(pico-sensors
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/sensors.c
  ${CMAKE_CURRENT_LIST_DIR}/sensors.h
)
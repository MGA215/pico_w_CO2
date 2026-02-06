add_library(pico-sensors INTERFACE)

add_subdirectory(ee895)
add_subdirectory(cdm7162)
add_subdirectory(sunrise)
add_subdirectory(sunlight)
add_subdirectory(scd30)
add_subdirectory(scd41)
add_subdirectory(cozir-lp3)
add_subdirectory(cm1107n)
add_subdirectory(ee872)
add_subdirectory(mux)
add_subdirectory(power)
add_subdirectory(ms5607)
add_subdirectory(hyt271)
add_subdirectory(generic_co2)

target_link_libraries(pico-sensors
  INTERFACE
  pico-ee895
  pico-ee872
  pico-cdm7162
  pico-sunrise
  pico-sunlight
  pico-scd30
  pico-scd41
  pico-cozir-lp3
  pico-cm1107n
  pico-ee872
  pico-mux
  pico-power
  pico-common
  pico-ms5607
  pico-hyt271
  pico-generic-co2
)

target_sources(pico-sensors
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/sensors.c
  ${CMAKE_CURRENT_LIST_DIR}/sensors.h
)
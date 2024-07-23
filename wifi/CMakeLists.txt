add_library(pico-wifi INTERFACE)

set_property(TARGET pico-wifi PROPERTY CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -g")

target_include_directories(pico-wifi
INTERFACE
)

target_sources(pico-wifi
INTERFACE
${CMAKE_CURRENT_LIST_DIR}/lwipopts.h
${CMAKE_CURRENT_LIST_DIR}/wifi.c
${CMAKE_CURRENT_LIST_DIR}/wifi.h
${CMAKE_CURRENT_LIST_DIR}/tcp_client.c
${CMAKE_CURRENT_LIST_DIR}/tcp_client.h
${CMAKE_CURRENT_LIST_DIR}/http.c
${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
${CMAKE_CURRENT_LIST_DIR}/websec.c
${CMAKE_CURRENT_LIST_DIR}/websec.h
${CMAKE_CURRENT_LIST_DIR}/base64.c
${CMAKE_CURRENT_LIST_DIR}/base64.h
)


target_link_libraries(pico-wifi
  INTERFACE
  pico_stdlib
  pico-common
  pico_lwip
  pico_cyw43_arch
  pico_cyw43_arch_lwip_threadsafe_background
  )

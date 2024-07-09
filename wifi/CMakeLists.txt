add_library(pico-wifi INTERFACE)

target_include_directories(pico-wifi
  INTERFACE
)

# set(WIFI_SSID ssid)
# set(WIFI_PASSWORD password)

# target_compile_definitions(pico-wifi INTERFACE
#         WIFI_SSID=\"${WIFI_SSID}\"
#         WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
# )


target_sources(pico-wifi
INTERFACE
${CMAKE_CURRENT_LIST_DIR}/lwipopts.h
${CMAKE_CURRENT_LIST_DIR}/wifi.c
${CMAKE_CURRENT_LIST_DIR}/wifi.h
${CMAKE_CURRENT_LIST_DIR}/tcp_client.c
${CMAKE_CURRENT_LIST_DIR}/tcp_client.h
${CMAKE_CURRENT_LIST_DIR}/http.c
${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)


target_link_libraries(pico-wifi
  INTERFACE
  pico_stdlib
  pico-common
  pico_lwip
  pico_cyw43_arch
  pico_cyw43_arch_lwip_threadsafe_background
)
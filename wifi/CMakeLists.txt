add_library(pico-wifi INTERFACE)

target_sources(pico-wifi
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/wifi_copy.c
  ${CMAKE_CURRENT_LIST_DIR}/http_copy.c
  ${CMAKE_CURRENT_LIST_DIR}/websec.c
  ${CMAKE_CURRENT_LIST_DIR}/base64.c
  ${CMAKE_CURRENT_LIST_DIR}/base64.h
  ${CMAKE_CURRENT_LIST_DIR}/tcp_client_copy.c
  ${CMAKE_CURRENT_LIST_DIR}/tcp_client_copy.h
  ${CMAKE_CURRENT_LIST_DIR}/tcp_server.c
  ${CMAKE_CURRENT_LIST_DIR}/tcp_server.h
  ${CMAKE_CURRENT_LIST_DIR}/websec.h
  ${CMAKE_CURRENT_LIST_DIR}/lwipopts.h
)
  
target_sources(pico-wifi INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/wifi_copy.h
  ${CMAKE_CURRENT_LIST_DIR}/../error_codes.h
)
  

target_link_libraries(pico-wifi INTERFACE 
  pico-common
  pico-cobs 
  pico_stdlib 
  pico_lwip
  pico-service_comm
  pico-uart
  pico_cyw43_arch_lwip_threadsafe_background
)

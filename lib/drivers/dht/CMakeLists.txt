add_library(pico_dht INTERFACE)

pico_generate_pio_header(pico_dht ${CMAKE_CURRENT_LIST_DIR}/dht.pio)

target_include_directories(pico_dht
    INTERFACE
    ./include)

target_sources(pico_dht
    INTERFACE
    dht.c
)

target_link_libraries(pico_dht
    INTERFACE
    hardware_clocks
    hardware_dma
    hardware_pio
)

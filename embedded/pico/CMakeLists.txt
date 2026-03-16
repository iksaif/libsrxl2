cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(srxl2_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

set(LIBSRXL2_DIR ../../libsrxl2)

# --- Sniffer (passive, packet decoder only, PIO RX) ---

add_executable(sniffer_pico
    sniffer_pico.c
    ${LIBSRXL2_DIR}/srxl2_packet.c
    ${LIBSRXL2_DIR}/srxl2_telemetry.c
)

pico_generate_pio_header(sniffer_pico ${CMAKE_CURRENT_LIST_DIR}/pio_uart.pio)
target_include_directories(sniffer_pico PRIVATE ${LIBSRXL2_DIR})
target_link_libraries(sniffer_pico pico_stdlib hardware_pio hardware_clocks)
pico_enable_stdio_usb(sniffer_pico 1)
pico_enable_stdio_uart(sniffer_pico 0)
pico_add_extra_outputs(sniffer_pico)

# --- Master (full state machine, PIO half-duplex) ---

add_executable(master_pico
    master_pico.c
    ${LIBSRXL2_DIR}/srxl2.c
    ${LIBSRXL2_DIR}/srxl2_packet.c
    ${LIBSRXL2_DIR}/srxl2_telemetry.c
)

pico_generate_pio_header(master_pico ${CMAKE_CURRENT_LIST_DIR}/pio_uart.pio)
target_include_directories(master_pico PRIVATE ${LIBSRXL2_DIR})
target_link_libraries(master_pico pico_stdlib hardware_pio hardware_clocks hardware_timer)
pico_enable_stdio_usb(master_pico 1)
pico_enable_stdio_uart(master_pico 0)
pico_add_extra_outputs(master_pico)

# --- Flight Controller (slave, PIO half-duplex single-wire UART) ---

add_executable(fc_pico
    fc_pico.c
    ${LIBSRXL2_DIR}/srxl2.c
    ${LIBSRXL2_DIR}/srxl2_packet.c
    ${LIBSRXL2_DIR}/srxl2_telemetry.c
)

pico_generate_pio_header(fc_pico ${CMAKE_CURRENT_LIST_DIR}/pio_uart.pio)
target_include_directories(fc_pico PRIVATE ${LIBSRXL2_DIR})
target_link_libraries(fc_pico pico_stdlib hardware_pio hardware_clocks hardware_timer)
pico_enable_stdio_usb(fc_pico 1)
pico_enable_stdio_uart(fc_pico 0)
pico_add_extra_outputs(fc_pico)

# --- Battery Sensor (slave, PIO half-duplex, FP_MAH telemetry) ---

add_executable(battery_pico
    battery_pico.c
    ${LIBSRXL2_DIR}/srxl2.c
    ${LIBSRXL2_DIR}/srxl2_packet.c
    ${LIBSRXL2_DIR}/srxl2_telemetry.c
)

pico_generate_pio_header(battery_pico ${CMAKE_CURRENT_LIST_DIR}/pio_uart.pio)
target_include_directories(battery_pico PRIVATE ${LIBSRXL2_DIR})
target_link_libraries(battery_pico pico_stdlib hardware_pio hardware_clocks hardware_timer)
pico_enable_stdio_usb(battery_pico 1)
pico_enable_stdio_uart(battery_pico 0)
pico_add_extra_outputs(battery_pico)

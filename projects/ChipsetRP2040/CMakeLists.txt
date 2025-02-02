cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ChipsetRP2040)

pico_sdk_init()

add_executable(ChipsetRP2040
    src/main.cc
    src/address_assignment.pio
    src/read_parallel.pio
    src/write_parallel.pio
)

pico_generate_pio_header(ChipsetRP2040 ${CMAKE_CURRENT_LIST_DIR}/src/address_assignment.pio)
pico_generate_pio_header(ChipsetRP2040 ${CMAKE_CURRENT_LIST_DIR}/src/read_parallel.pio)
pico_generate_pio_header(ChipsetRP2040 ${CMAKE_CURRENT_LIST_DIR}/src/write_parallel.pio)

target_link_libraries(ChipsetRP2040 pico_stdlib hardware_pio)

pico_add_extra_outputs(ChipsetRP2040)
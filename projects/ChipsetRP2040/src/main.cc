#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "address_assignment.pio.h"
#include "read_parallel.pio.h"
#include "write_parallel.pio.h"

constexpr auto SA5 = 0;
constexpr auto SA4 = 1;
constexpr auto SA3 = 2;
constexpr auto SA2 = 3;
constexpr auto SA1 = 4;
constexpr auto SA0 = 5;
constexpr auto SOE = 6;
constexpr auto SWR = 7;
constexpr auto SD0 = 8;
constexpr auto SD1 = 9;
constexpr auto SD2 = 10;
constexpr auto SD3 = 11;
constexpr auto SD4 = 12;
constexpr auto SD5 = 13;
constexpr auto SD6 = 14;
constexpr auto SD7 = 15;

PIO pio = pio0;
uint address_sm = 0;
uint read_sm = 0;
uint write_sm = 0;

void setup() {
    // Initialize control pins
    pinMode(SOE, OUTPUT);
    pinMode(SWR, OUTPUT);

    // Initialize data pins
    pinMode(SD0, OUTPUT);
    pinMode(SD1, OUTPUT);
    pinMode(SD2, OUTPUT);
    pinMode(SD3, OUTPUT);
    pinMode(SD4, OUTPUT);
    pinMode(SD5, OUTPUT);
    pinMode(SD6, OUTPUT);
    pinMode(SD7, OUTPUT);

    // Set initial states
    digitalWrite(SOE, HIGH);
    digitalWrite(SWR, HIGH);
    digitalWrite(SD0, LOW);
    digitalWrite(SD1, LOW);
    digitalWrite(SD2, LOW);
    digitalWrite(SD3, LOW);
    digitalWrite(SD4, LOW);
    digitalWrite(SD5, LOW);
    digitalWrite(SD6, LOW);
    digitalWrite(SD7, LOW);

    // Initialize PIO for address assignment
    auto address_offset = pio_add_program(pio, &address_assignment_program);
    address_sm = pio_claim_unused_sm(pio, true);
    pio_sm_config address_config = address_assignment_program_get_default_config(address_offset);
    sm_config_set_out_pins(&address_config, SA0, 6);
    sm_config_set_out_shift(&address_config, false, true, 32);
    pio_sm_set_consecutive_pindirs(pio, address_sm, SA0, 6, true);
    pio_sm_init(pio, address_sm, address_offset, &address_config);
    pio_sm_set_enabled(pio, address_sm, true);

    // Initialize PIO for reading
    auto read_offset = pio_add_program(pio, &read_parallel_program);
    read_sm = pio_claim_unused_sm(pio, true);
    pio_sm_config read_config = read_parallel_program_get_default_config(read_offset);
    sm_config_set_in_pins(&read_config, SD0);
    sm_config_set_in_shift(&read_config, false, true, 32);
    pio_sm_set_consecutive_pindirs(pio, read_sm, SD0, 8, false);
    pio_sm_init(pio, read_sm, read_offset, &read_config);
    pio_sm_set_enabled(pio, read_sm, true);

    // Initialize PIO for writing
    auto write_offset = pio_add_program(pio, &write_parallel_program);
    write_sm = pio_claim_unused_sm(pio, true);
    pio_sm_config write_config = write_parallel_program_get_default_config(write_offset);
    sm_config_set_out_pins(&write_config, SD0, 8);
    sm_config_set_out_shift(&write_config, false, true, 32);
    pio_sm_set_consecutive_pindirs(pio, write_sm, SD0, 8, true);
    pio_sm_init(pio, write_sm, write_offset, &write_config);
    pio_sm_set_enabled(pio, write_sm, true);
}

void loop() {
    // Example: Write data 0x55 to address 0x03
    writeParallel(0x03, 0x55);

    // Example: Read data from address 0x03
    uint8_t data = readParallel(0x03);
    delay(1000);
}

void setAddress(uint8_t address) {
    pio_sm_put_blocking(pio, address_sm, address);
}

void writeParallel(uint8_t address, uint8_t data) {
    // Set address using PIO
    setAddress(address);

    // Set data pins to output
    pio_sm_set_consecutive_pindirs(pio, write_sm, SD0, 8, true);

    // Write data using PIO
    pio_sm_put_blocking(pio, write_sm, data);
    digitalWrite(SWR, LOW); // Enable write
    delayMicroseconds(1); // Small delay for write operation
    digitalWrite(SWR, HIGH); // Disable write
}

uint8_t readParallel(uint8_t address) {
    // Set address using PIO
    setAddress(address);

    // Set data pins to input
    pio_sm_set_consecutive_pindirs(pio, read_sm, SD0, 8, false);

    // Read data using PIO
    digitalWrite(SOE, LOW); // Enable read
    delayMicroseconds(1); // Small delay for read operation
    uint8_t data = pio_sm_get_blocking(pio, read_sm);
    digitalWrite(SOE, HIGH); // Disable read

    return data;
}
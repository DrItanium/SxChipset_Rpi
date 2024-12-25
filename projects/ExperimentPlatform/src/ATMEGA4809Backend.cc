#ifdef ARDUINO_AVR_ATmega4809
#include <Arduino.h>
#include <microshell.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Concepts.h"

struct ush_node_object root;
const struct ush_file_descriptor rootFiles[] = { };
struct ush_node_object dev;
const struct ush_file_descriptor devFiles[] = {
    {
        .name = "time", 
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr, 
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { return sendDword(PASS_FILE_DESCRIPTOR_ARGS, data, millis()); },
    },
    {
        .name = "micros",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr, 
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { return sendDword(PASS_FILE_DESCRIPTOR_ARGS, data, micros()); },
    },
#ifdef GPIOR0
    ByteFile("gpr0", GPIOR0),
#endif
#ifdef GPIOR1
    ByteFile("gpr1", GPIOR1),
#endif
#ifdef GPIOR2
    ByteFile("gpr2", GPIOR2),
#endif
#ifdef GPIOR3
    ByteFile("gpr3", GPIOR3),
#endif
#define AnalogFile(n, reg) { \
        .name = n ,  \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { return sendWord(PASS_FILE_DESCRIPTOR_ARGS, data, analogRead( reg )); }, \
    } 
#define X(pin) AnalogFile(#pin, pin)
#include <AnalogPins.def>
#undef X
#undef AnalogFile
};
#define DefVPort(id) \
const struct ush_file_descriptor gpioVPort ## id ## Files [] = { \
    ByteFile("in", VPORT ## id . IN ), \
    ByteFile("out", VPORT ## id . OUT ), \
    ByteFile("dir", VPORT ## id . DIR ), \
    ByteFile("intflags", VPORT ## id . INTFLAGS ), \
}; \
struct ush_node_object gpioVPort ## id ## Dir 
#define X(id) DefVPort(id);
#include <AVRPorts.def>
#undef X
#undef DefVPort

struct ush_node_object specificCmd;
const struct ush_file_descriptor specificCmdFiles[] = {
    {
        .name = "analogRead",
        .description = nullptr,
        .help = "usage: analogRead A[0-...]\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            if (argc != 2) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                return;
            }
            auto* arg1 = argv[1];
#define X(letter) \
            if (strcmp(arg1, #letter ) == 0) {  \
                ush_printf(self, "%d\r\n", analogRead ( letter ) ); \
                return; \
            } 
#include <AnalogPins.def>
#undef X
            // fallback condition
            ush_printf(self, "Unknown analog device %s\r\n", arg1);
        },
    },
    {
        .name = "lsvport",
        .description = nullptr,
        .help = "usage: lsvport ?port-letter\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            if (argc != 2) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                return;
            }
            auto* arg1 = argv[1];
#define X(letter) \
            if (strcmp(arg1, #letter ) == 0) {  \
                ush_printf(self, "\tIN: 0x%x\r\n", VPORT ## letter . IN); \
                ush_printf(self, "\tOUT: 0x%x\r\n", VPORT ## letter . OUT); \
                ush_printf(self, "\tDIR: 0x%x\r\n", VPORT ## letter . DIR); \
                ush_printf(self, "\tINTFLAGS: 0x%x\r\n", VPORT ## letter . INTFLAGS); \
                return; \
            } 
#include <AVRPorts.def>
#undef X
            ush_printf(self, "Unknown port %s\r\n", arg1);
        },
    },
    {
        .name = "lsport",
        .description = nullptr,
        .help = "usage: lsvport ?port-letter\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            if (argc != 2) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                return;
            }
            auto* arg1 = argv[1];
#define Y(letter, idx) ush_printf(self, "\tPIN%dCTRL: 0x%x\r\n", idx, PORT ## letter . PIN ## idx ## CTRL )
#define X(letter) \
            if (strcmp(arg1, #letter ) == 0) {  \
                ush_printf(self, "\tIN: 0x%x\r\n", PORT ## letter . IN); \
                ush_printf(self, "\tOUT: 0x%x\r\n", PORT ## letter . OUT); \
                ush_printf(self, "\tDIR: 0x%x\r\n", PORT ## letter . DIR); \
                ush_printf(self, "\tINTFLAGS: 0x%x\r\n", PORT ## letter . INTFLAGS); \
                ush_printf(self, "\tPORTCTRL: 0x%x\r\n", PORT ## letter . PORTCTRL); \
                Y(letter, 0); \
                Y(letter, 1); \
                Y(letter, 2); \
                Y(letter, 3); \
                Y(letter, 4); \
                Y(letter, 5); \
                Y(letter, 6); \
                Y(letter, 7); \
                return; \
            } 
#include <AVRPorts.def>
#undef X
#undef Y
            ush_printf(self, "Unknown port %s\r\n", arg1);
        },
    },
};

void
configureFileSystem(ush_object& ush) {
    ush_commands_add(&ush, &specificCmd, specificCmdFiles, NELEM(specificCmdFiles));
    ush_node_mount(&ush, "/", &root, rootFiles, NELEM(rootFiles));
    ush_node_mount(&ush, "/dev", &dev, devFiles, NELEM(devFiles));
#define RegisterPort(path, id) \
    ush_node_mount(&ush, path , & gpioVPort ## id ## Dir , gpioVPort ## id ## Files , NELEM( gpioVPort ## id ## Files ))
#define X(index) RegisterPort( "/dev/vport" #index , index );
#include <AVRPorts.def>
#undef X
#undef RegisterPort
}
void
targetSpecificSetup() {
#ifdef PLATFORM_CHIPSET
    // want to be able to talk to the ClockChip
    SPI.begin();
    Wire.begin();
    if (SD.begin(PIN_PB0)) {
        Serial.println(F("SD Card Found!"));
        sdCardFound = true;
    } else {
        Serial.println(F("SD Card Not Found!"));
        sdCardFound = false;
    }
#endif
}
#endif

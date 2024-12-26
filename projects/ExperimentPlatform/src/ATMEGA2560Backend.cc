#ifdef ARDUINO_AVR_ATmega2560
#include <Arduino.h>
#include <microshell.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Concepts.h"
#ifdef PLATFORM_CHIPSET
#include <SD.h>
#endif

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
#define DefTimer(index) \
const struct ush_file_descriptor timer ## index ## Files [] = { \
    ByteFile("tccra", TCCR ## index ## A ), \
    ByteFile("tccrb", TCCR ## index ## B ), \
    ByteFile("tccrc", TCCR ## index ## C ), \
    ByteFile("tifr", TIFR ## index ), \
    WordFile("tcnt", TCNT ## index ), \
    WordFile("icr", ICR ## index ), \
    WordFile("ocra", OCR ## index ## A ), \
    WordFile("ocrb", OCR ## index ## B ), \
    WordFile("ocrc", OCR ## index ## C ), \
}; \
struct ush_node_object timer ## index ## Dir 
#ifdef TCCR1A
DefTimer(1);
#define HAVE_TIMER1
#endif
#ifdef TCCR3A
DefTimer(3);
#define HAVE_TIMER3
#endif
#ifdef TCCR4A
DefTimer(4);
#define HAVE_TIMER4
#endif
#ifdef TCCR5A
DefTimer(5);
#define HAVE_TIMER5
#endif
#undef DefTimer
#define DefPort(id) \
const struct ush_file_descriptor gpioPort ## id ## Files [] = { \
    ByteFile("in", PIN ## id ), \
    ByteFile("out", PORT ## id ), \
    ByteFile("dir", DDR ## id ), \
}; \
struct ush_node_object gpioPort ## id ## Dir 
#define X(id) DefPort(id);
#include <AVRPorts.def>
#undef X
#undef DefPort

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
        .name = "lsport",
        .description = nullptr,
        .help = "usage: lsport {A-L}\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            if (argc != 2) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                return;
            }
            auto* arg1 = argv[1];
#define X(letter) \
            if (strcmp(arg1, #letter ) == 0) {  \
                ush_printf(self, "\tIN: 0x%x\r\n", PIN ## letter); \
                ush_printf(self, "\tOUT: 0x%x\r\n", PORT ## letter); \
                ush_printf(self, "\tDIR: 0x%x\r\n", DDR ## letter); \
                return; \
            } 
            X(A)
            X(B)
            X(C)
            X(D)
            X(E)
            X(F)
            X(G)
            X(H)
            X(J)
            X(K)
            X(L)
#undef X
            ush_printf(self, "Unknown port %s\r\n", arg1);
        },
    },
};

void
configureFileSystem(ush_object& ush) {
    ush_commands_add(&ush, &specificCmd, specificCmdFiles, NELEM(specificCmdFiles));
    ush_node_mount(&ush, "/", &root, rootFiles, NELEM(rootFiles));
    ush_node_mount(&ush, "/dev", &dev, devFiles, NELEM(devFiles));
#ifdef HAVE_TIMER1
    ush_node_mount(&ush, "/dev/timer1", &timer1Dir, timer1Files, NELEM(timer1Files));
#endif
#ifdef HAVE_TIMER3
    ush_node_mount(&ush, "/dev/timer3", &timer3Dir, timer3Files, NELEM(timer3Files));
#endif
#ifdef HAVE_TIMER4
    ush_node_mount(&ush, "/dev/timer4", &timer4Dir, timer4Files, NELEM(timer4Files));
#endif
#ifdef HAVE_TIMER5
    ush_node_mount(&ush, "/dev/timer5", &timer5Dir, timer5Files, NELEM(timer5Files));
#endif
#define RegisterPort(path, id) \
    ush_node_mount(&ush, path , & gpioPort ## id ## Dir , gpioPort ## id ## Files , NELEM( gpioPort ## id ## Files ))
#define X(index) RegisterPort( "/dev/port" #index , index );
#include <AVRPorts.def>
#undef X
#undef RegisterPort
}
bool sdCardFound = false;
void
targetSpecificSetup() {
    Serial.begin(115200);
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

// taken from the BasicExample
int 
ushRead(struct ush_object* self, char* ch) {
    if (Serial.available() > 0) {
        *ch = Serial.read();
        return 1;
    }
    return 0;
}

int 
ushWrite(struct ush_object* self, char ch) {
    return (Serial.write(ch) == 1);
}

#endif // end defined(ARDUINO_AVR_ATmega2560)

/*
i960SxChipset_RPi
Copyright (c) 2022-2024, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <microshell.h>
#include <STM32SD.h>

#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN 2
#endif

Sd2Card card;
SdFatFs fatFs;
bool sdcardFound = false;

#define FILE_DESCRIPTOR_ARGS struct ush_object* self, struct ush_file_descriptor const * file
#define PASS_FILE_DESCRIPTOR_ARGS self, file
#define ByteFile(n, reg) { \
        .name = n , \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { \
            return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, reg ); \
        }, \
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) noexcept { \
            uint8_t value = 0; \
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, value)) { \
                reg = value; \
            } \
        }, \
    }

#define ByteFile_RO(n, reg) { \
        .name = n , \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { \
            return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, reg ); \
        }, \
        .set_data = nullptr, \
    }

#define WordFile(n, reg) { \
        .name = n , \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendWord(PASS_FILE_DESCRIPTOR_ARGS, data, reg ); }, \
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) {  \
            uint16_t result = 0; \
            if (retrieveWord(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) { \
                reg = result; \
            } \
        } \
    }

#define NELEM(obj) (sizeof(obj) / sizeof(obj[0]))

unsigned int sendDword(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint32_t value) noexcept {
    static char buf[16];
    snprintf(buf, sizeof(buf), "%lu\n", value);
    buf[sizeof(buf) - 1] = 0;
    *data = (uint8_t*)buf;
    return strlen((char*)(*data));
}
unsigned int sendWord(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint16_t value) noexcept {
    static char buf[16];
    snprintf(buf, sizeof(buf), "%u\n", value);
    buf[sizeof(buf) - 1] = 0;
    *data = (uint8_t*)buf;
    return strlen((char*)(*data));
}
unsigned int sendByte(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint8_t value) noexcept {
    return sendWord(PASS_FILE_DESCRIPTOR_ARGS, data, value);
}
bool retrieveWord(FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size, uint16_t& result) noexcept {
    if (size < 1) {
        return false;
    }
    (void)sscanf((char*)data, "%u", &result);
    return true;
}

bool retrieveByte(FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size, uint8_t& result) noexcept {
    uint16_t value = 0;
    if (retrieveWord(PASS_FILE_DESCRIPTOR_ARGS, data, size, value)) {
        result = value;
        return true;
    }
    return false;
}

extern int ushRead(struct ush_object* self, char* ch);
extern int ushWrite(struct ush_object* self, char ch);



constexpr auto BufferInSize = 256;
constexpr auto BufferOutSize = 256;
constexpr auto PathMaxSize = 128;

char ushInBuf[BufferInSize];
char ushOutBuf[BufferOutSize];
uint32_t currentRandomSeed = 0;
char hostname[16] = "stm32_u575";

struct ush_object ush;
const struct ush_io_interface ushInterface = {
    .read = ushRead,
    .write = ushWrite,
};
const struct ush_descriptor ush_desc = {
    .io = &ushInterface,
    .input_buffer = ushInBuf,
    .input_buffer_size = sizeof(ushInBuf),
    .output_buffer = ushOutBuf,
    .output_buffer_size = sizeof(ushOutBuf),
    .path_max_length = PathMaxSize,
    .hostname = hostname,
};

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
#define X(id, str, letter) AnalogFile(str, id),
#include <AnalogPins.def>
#undef X
#undef AnalogFile
};
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
#define X(id, str, letter) \
            if (strcmp(arg1, str ) == 0) {  \
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
        .name = "digitalRead",
        .description = nullptr,
        .help = "usage: digitalRead ?idx\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            switch (argc) {
                case 2: {
                            long idx = 0;
                            (void)sscanf(argv[1], "%ld", &idx);
                            ush_printf(self, "%s\r\n", digitalRead(idx) ? "HIGH" : "LOW");
                            break;
                        }
                default:
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                    break;
            }
        },
    },
    {
        .name = "rand",
        .description = nullptr,
        .help = "usage: rand [max] [min] \r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            switch (argc) {
                case 1:
                    ush_printf(self, "%ld\r\n", random());
                    break;
                case 2: {
                            long max = 0;
                            (void)sscanf(argv[1], "%ld", &max);
                            ush_printf(self, "%ld\r\n", random(max));
                            break;
                        }
                default: {
                             long max = 0;
                             long min = 0;
                             (void)sscanf(argv[1], "%ld", &max);
                             (void)sscanf(argv[2], "%ld", &min);
                             ush_printf(self, "%ld\r\n", random(min, max));
                             break;
                          }


            }
        },
    },
    {
        .name = "digitalWrite",
        .description = nullptr,
        .help = "usage: digitalWrite [pin] [0|1]\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            switch (argc) {
                case 0:
                case 1:
                case 2:
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                    break;
                default: {
                             long pin = 0;
                             char* value = argv[2];
                             (void)sscanf(argv[1], "%ld", &pin);
#define X(cmp, val) \
                             if (strcmp(value, cmp) == 0) { \
                                 digitalWrite(pin, val);  \
                                 break; \
                             }
                             X("0", LOW);
                             X("LOW", LOW);
                             X("low", LOW);
                             X("false", LOW);
                             X("f", LOW);
                             X("F", LOW);
                             X("HIGH", HIGH);
                             X("high", HIGH);
                             X("1", HIGH);
                             X("true", HIGH);
                             X("TRUE", HIGH);
                             X("t", HIGH);
                             X("T", HIGH);
                             X("True", HIGH);
                             X("False", LOW);
#undef X
                             ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                             break;
                          }
            }
        },
    },
    {
        .name = "pinMode",
        .description = nullptr,
        .help = "usage: pinMode [pin] [INPUT|OUTPUT|INPUT_PULLUP]\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            switch (argc) {
                case 0:
                case 1:
                case 2:
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                    break;
                default: {
                             long pin = 0;
                             (void)sscanf(argv[1], "%ld", &pin);
                             char* direction = argv[2];
#define X(cmp, val) \
                             if (strcmp(direction, cmp) == 0) { \
                                 pinMode(pin, val);  \
                                 break; \
                             }
                             X("INPUT", INPUT);
                             X("input", INPUT);
                             X("in", INPUT);
                             X("IN", INPUT);
                             X("INPUT_PULLUP", INPUT_PULLUP);
                             X("INPUT-PULLUP", INPUT_PULLUP);
                             X("input-pullup", INPUT_PULLUP);
                             X("input_pullup", INPUT_PULLUP);
                             X("OUTPUT", OUTPUT);
                             X("output", OUTPUT);
                             X("out", OUTPUT);
                             X("OUT", OUTPUT);
#undef X
                             ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                             break;
                          }
            }
        },
    },
    {
        .name = "pinInfo",
        .description = nullptr,
        .help = "usage: pinInfo\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            ush_printf(self, "Digital Pin Count: %d\r\n", NUM_DIGITAL_PINS);
            ush_printf(self, "Analog Input Count: %d\r\n", NUM_ANALOG_INPUTS);

        },
    },
    {
        .name = "ledColor",
        .description = nullptr,
        .help = "usage: ledColor ?r ?g ?b\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
#define X(pin, arg, cmp, val) \
                             if (strcmp(arg, cmp) == 0) { \
                                 digitalWrite(pin, val);  \
                                 break; \
                             }
            switch (argc) {
                case 1:
                case 2:
                case 3:
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                    break;
                default:
                    X(LED_RED, argv[1], "0", LOW);
                    X(LED_RED, argv[1], "1", HIGH);
                    X(LED_GREEN, argv[2], "0", LOW);
                    X(LED_GREEN, argv[2], "1", HIGH);
                    X(LED_BLUE, argv[3], "0", LOW);
                    X(LED_BLUE, argv[3], "1", HIGH);
                    break;
            }
#undef X
        },
    },
    {
        .name = "lsperiph",
        .description = nullptr,
        .help = "usage: pinInfo\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            ush_printf(self, "Digital Pin Count: %d\r\n", NUM_DIGITAL_PINS);
            ush_printf(self, "Analog Input Count: %d\r\n", NUM_ANALOG_INPUTS);

        },
    },
    {
        .name = "sdcard_info",
        .description = nullptr,
        .help = "usage: sdcard_info\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            if (sdcardFound) {
                auto displayCardType = [&self](const char* string) {
                    ush_printf(self, "Card Type: %s\n", string);
                };
                switch (card.type()) {
                    case SD_CARD_TYPE_SD1:
                        displayCardType("SD1");
                        break;
                    case SD_CARD_TYPE_SD2:
                        displayCardType("SD2");
                        break;
                    case SD_CARD_TYPE_SDHC:
                        displayCardType("SDHC");
                        break;
                    default:
                        displayCardType("Unknown");
                        break;
                }
            } else {
                ush_printf(self, "NO SDCARD Inserted!\r\n");
            }

        },
    }
};




uint32_t
computeRandomSeed() {
    uint32_t x = 0;
#define X(id, str, letter) x += analogRead ( id );
#include <AnalogPins.def>
#undef X
    return x;
}
extern void targetSpecificSetup();
void 
setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.print("\nInitializing SD Card...");
    card.setDx(PC_8, PC_9, PC_10, PC_11);
    card.setCK(PC_12);
    card.setCMD(PD_2);
    while (!card.init(SD_DETECT_PIN, HIGH)) {
        delay(10);
    }
    sdcardFound = true;
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
    // setup a random source
    currentRandomSeed = computeRandomSeed();

    randomSeed(currentRandomSeed);
    ush_init(&ush, &ush_desc);
    ush_commands_add(&ush, &specificCmd, specificCmdFiles, NELEM(specificCmdFiles));
    ush_node_mount(&ush, "/", &root, rootFiles, NELEM(rootFiles));
    ush_node_mount(&ush, "/dev", &dev, devFiles, NELEM(devFiles));
}


void 
loop() {
    ush_service(&ush);
}

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


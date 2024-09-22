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
#include <microshell.h>


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


constexpr auto BufferInSize = 128;
constexpr auto BufferOutSize = 128;
constexpr auto PathMaxSize = 128;

char ushInBuf[BufferInSize];
char ushOutBuf[BufferOutSize];
uint32_t currentRandomSeed = 0;
char hostname[16] = "atmega2560";

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


const struct ush_file_descriptor rootFiles[] = { };




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
    ByteFile("gpr0", GPIOR0),
    ByteFile("gpr1", GPIOR1),
    ByteFile("gpr2", GPIOR2),
#define AnalogFile(n, reg) { \
        .name = n ,  \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { return sendWord(PASS_FILE_DESCRIPTOR_ARGS, data, analogRead( reg )); }, \
    } 
    AnalogFile("ana0", A0),
    AnalogFile("ana1", A1),
    AnalogFile("ana2", A2),
    AnalogFile("ana3", A3),
    AnalogFile("ana4", A4),
    AnalogFile("ana5", A5),
    AnalogFile("ana6", A6),
    AnalogFile("ana7", A7),
    AnalogFile("ana8", A8),
    AnalogFile("ana9", A9),
    AnalogFile("ana10", A10),
    AnalogFile("ana11", A11),
    AnalogFile("ana12", A12),
    AnalogFile("ana13", A13),
    AnalogFile("ana14", A14),
    AnalogFile("ana15", A15),
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
DefTimer(1);
DefTimer(3);
DefTimer(4);
DefTimer(5);
#undef DefTimer
#define DefPort(id) \
const struct ush_file_descriptor gpioPort ## id ## Files [] = { \
    ByteFile("in", PIN ## id ), \
    ByteFile("out", PORT ## id ), \
    ByteFile("dir", DDR ## id ), \
}; \
struct ush_node_object gpioPort ## id ## Dir 
DefPort(A);
DefPort(B);
DefPort(C);
DefPort(D);
DefPort(E);
DefPort(F);
DefPort(G);
DefPort(H);
DefPort(J);
DefPort(K);
DefPort(L);
#undef DefPort
const struct ush_file_descriptor cmdFiles[] = {
    {
        .name = "analogRead",
        .description = nullptr,
        .help = "usage: analogRead A[0-15]\r\n",
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
            X(A0);
            X(A1);
            X(A2);
            X(A3);
            X(A4);
            X(A5);
            X(A6);
            X(A7);
            X(A8);
            X(A9);
            X(A10);
            X(A11);
            X(A12);
            X(A13);
            X(A14);
            X(A15);
#undef X
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
};


struct ush_node_object root;
struct ush_node_object dev;
struct ush_node_object cmd;



void 
setup() {
    Serial.begin(115200);
    // setup a random source
    currentRandomSeed = analogRead(A0);
#define X(id) currentRandomSeed += analogRead ( id ) 
    X(A1);
    X(A2);
    X(A3);
    X(A4);
    X(A5);
    X(A6);
    X(A7);
    X(A8);
    X(A9);
    X(A10);
    X(A11);
    X(A12);
    X(A13);
    X(A14);
    X(A15);
#undef X
    randomSeed(currentRandomSeed);
    Wire.begin();
    SPI.begin();
    ush_init(&ush, &ush_desc);
#define NELEM(obj) (sizeof(obj) / sizeof(obj[0]))
    ush_commands_add(&ush, &cmd, cmdFiles, NELEM(cmdFiles));
    ush_node_mount(&ush, "/", &root, rootFiles, NELEM(rootFiles));
    ush_node_mount(&ush, "/dev", &dev, devFiles, NELEM(devFiles));
    ush_node_mount(&ush, "/dev/timer1", &timer1Dir, timer1Files, NELEM(timer1Files));
    ush_node_mount(&ush, "/dev/timer3", &timer3Dir, timer3Files, NELEM(timer3Files));
    ush_node_mount(&ush, "/dev/timer4", &timer4Dir, timer4Files, NELEM(timer4Files));
    ush_node_mount(&ush, "/dev/timer5", &timer5Dir, timer5Files, NELEM(timer5Files));
#define RegisterPort(path, id) \
    ush_node_mount(&ush, path , & gpioPort ## id ## Dir , gpioPort ## id ## Files , NELEM( gpioPort ## id ## Files ))
    RegisterPort("/dev/porta", A);
    RegisterPort("/dev/portb", B);
    RegisterPort("/dev/portc", C);
    RegisterPort("/dev/portd", D);
    RegisterPort("/dev/porte", E);
    RegisterPort("/dev/portf", F);
    RegisterPort("/dev/portg", G);
    RegisterPort("/dev/porth", H);
    RegisterPort("/dev/portj", J);
    RegisterPort("/dev/portk", K);
    RegisterPort("/dev/portl", L);
#undef RegisterPort
#undef NELEM
}

void 
loop() {
    ush_service(&ush);
}



#undef WordFile
#undef ByteFile
#undef PASS_FILE_DESCRIPTOR_ARGS
#undef FILE_DESCRIPTOR_ARGS

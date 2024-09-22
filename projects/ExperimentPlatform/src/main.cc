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
    .hostname = "experimental2560",
};


// info file get data callback
size_t
fixedFileDataCallback(FILE_DESCRIPTOR_ARGS, uint8_t** data, const char* info) noexcept {
    *data = (uint8_t*)info;
    return strlen(info);
}


// time file get data callback
size_t
timeGetDataCallback(FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
    static char timeBuf[16];
    // read current time
    auto currentTime = millis();
    // convert
    snprintf(timeBuf, sizeof(timeBuf), "%ld\r\n", currentTime);
    timeBuf[sizeof(timeBuf) -1] = 0;
    *data = (uint8_t*)timeBuf;
    return strlen((char*)(*data));
}

const struct ush_file_descriptor rootFiles[] = {
    {
        .name = "info.txt", 
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            return fixedFileDataCallback(PASS_FILE_DESCRIPTOR_ARGS, data, "Teensy 4.1 System Chip\n");
        },
    }
};

const struct ush_file_descriptor cmdToggleLED{
    .name = "toggle",
    .description = "toggle led",
    .help = "usage: toggle\n",
    .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    },
};

const struct ush_file_descriptor cmdSetLED {
        .name = "set", 
        .description = "set led",
        .help = "usage: set {0,1}\r\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            if (argc != 0) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                return;
            }

            if (strcmp(argv[1], "1") == 0) {
                digitalWrite(LED_BUILTIN, HIGH);
            } else if (strcmp(argv[1], "0") == 0) {
                digitalWrite(LED_BUILTIN, LOW);
            } else {
                // return predefined error message
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
                return;
            }
        },
};

const struct ush_file_descriptor cmdFeedbackTest {
    .name = "feedback",
    .description = "display what was passed in",
    .help = "usage: feedback ...\n",
    .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
        for (int i = 0; i < argc; ++i) {
            ush_printf(self, "\t%d: %s\n", i, argv[i]);
        }
    },
};


const struct ush_file_descriptor binFiles[] = {
    cmdToggleLED,
    cmdSetLED,
    cmdFeedbackTest,
};


const struct ush_file_descriptor devFiles[] = {
    {
        .name = "led", 
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            // read current led state
            bool state = digitalRead(LED_BUILTIN);
            return sendWord(PASS_FILE_DESCRIPTOR_ARGS, data, state ? 1 : 0);
        },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) noexcept {
            if (size < 1) {
                return;
            }
            
            if (data[0] == '1') {
                digitalWrite(LED_BUILTIN, HIGH);
            } else if (data[0] == '0') {
                digitalWrite(LED_BUILTIN, LOW);
            }
        },
    },
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
        .name = "reboot",
        .description = "reboot device",
        .help = nullptr,
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            ush_print(self, "error: reboot not supported");
        },
    },
};


struct ush_node_object root;
struct ush_node_object dev;
struct ush_node_object bin;
struct ush_node_object cmd;



void 
setup() {
    Serial.begin(115200);
    ush_init(&ush, &ush_desc);
#define NELEM(obj) (sizeof(obj) / sizeof(obj[0]))
    ush_commands_add(&ush, &cmd, cmdFiles, NELEM(cmdFiles));
    ush_node_mount(&ush, "/", &root, rootFiles, NELEM(rootFiles));
    ush_node_mount(&ush, "/bin", &bin, binFiles, NELEM(binFiles));
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

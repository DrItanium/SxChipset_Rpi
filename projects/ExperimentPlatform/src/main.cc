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
#define FILE_DESCRIPTOR_ARGS struct ush_object* self, struct ush_file_descriptor const * file
#define PASS_FILE_DESCRIPTOR_ARGS self, file


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
            // return pointer to data
            *data = (uint8_t*)((state) ? "1\n" : "0\n");
            // return data size
            return strlen((char*)(*data));
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
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char timeBuf[16];
            // read current time
            auto currentTime = millis();
            // convert
            snprintf(timeBuf, sizeof(timeBuf), "%ld\r\n", currentTime);
            timeBuf[sizeof(timeBuf) -1] = 0;
            *data = (uint8_t*)timeBuf;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "micros",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr, 
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char timeBuf[16];
            // read current time
            auto currentTime = micros();
            // convert
            snprintf(timeBuf, sizeof(timeBuf), "%ld\r\n", currentTime);
            timeBuf[sizeof(timeBuf) -1] = 0;
            *data = (uint8_t*)timeBuf;
            return strlen((char*)(*data));
        },
    },
};
unsigned int sendUint16(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint16_t value) noexcept {
    static char buf[16];
    snprintf(buf, sizeof(buf), "%d\n", value);
    buf[sizeof(buf) - 1] = 0;
    *data = (uint8_t*)buf;
    return strlen((char*)(*data));
}
unsigned int sendByte(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint8_t value) noexcept {
    return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, value);
}
bool retrieveUint16(FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size, uint16_t& result) noexcept {
    if (size < 1) {
        return false;
    }
    (void)sscanf((char*)data, "%u", &result);
    return true;
}

bool retrieveByte(FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size, uint8_t& result) noexcept {
    uint16_t value = 0;
    if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, value)) {
        result = value;
        return true;
    }
    return false;
}
const struct ush_file_descriptor timer1Files[] = {
    {
        .name = "tccra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR1A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR1A = result;
            }
        }
    },
    {
        .name = "tccrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR1B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR1B = result;
            }
        }
    },
    {
        .name = "tccrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR1C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR1C = result;
            }
        }
    },
    {
        .name = "tcnt",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, TCNT1); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCNT1 = result;
            }
        }
    },
    {
        .name = "icr",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, ICR1); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                ICR1 = result;
            }
        }
    },

    {
        .name = "ocra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR1A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR1A = result;
            }
        }
    },
    {
        .name = "ocrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR1B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR1B = result;
            }
        }
    },
    {
        .name = "ocrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR1C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR1C = result;
            }
        }
    },
};
const struct ush_file_descriptor timer3Files[] = {
    {
        .name = "tccra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR3A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR3A = result;
            }
        }
    },
    {
        .name = "tccrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR3B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR3B = result;
            }
        }
    },
    {
        .name = "tccrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR3C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR3C = result;
            }
        }
    },
    {
        .name = "tcnt",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, TCNT3); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCNT3 = result;
            }
        }
    },
    {
        .name = "icr",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, ICR3); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                ICR3 = result;
            }
        }
    },

    {
        .name = "ocra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR3A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR3A = result;
            }
        }
    },
    {
        .name = "ocrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR3B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR3B = result;
            }
        }
    },
    {
        .name = "ocrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR3C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR3C = result;
            }
        }
    },
};
const struct ush_file_descriptor timer4Files[] = {
    {
        .name = "tccra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR4A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR4A = result;
            }
        }
    },
    {
        .name = "tccrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR4B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR4B = result;
            }
        }
    },
    {
        .name = "tccrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR4C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR4C = result;
            }
        }
    },
    {
        .name = "tcnt",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, TCNT4); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCNT4 = result;
            }
        }
    },
    {
        .name = "icr",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, ICR4); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                ICR4 = result;
            }
        }
    },

    {
        .name = "ocra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR4A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR4A = result;
            }
        }
    },
    {
        .name = "ocrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR4B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR4B = result;
            }
        }
    },
    {
        .name = "ocrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR4C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR4C = result;
            }
        }
    },
};

const struct ush_file_descriptor timer5Files[] = {
    {
        .name = "tccra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR5A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR5A = result;
            }
        }
    },
    {
        .name = "tccrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR5B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR5B = result;
            }
        }
    },
    {
        .name = "tccrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, TCCR5C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint8_t result = 0;
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCCR5C = result;
            }
        }
    },
    {
        .name = "tcnt",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, TCNT5); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                TCNT5 = result;
            }
        }
    },
    {
        .name = "icr",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, ICR5); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                ICR5 = result;
            }
        }
    },

    {
        .name = "ocra",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR5A); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR5A = result;
            }
        }
    },
    {
        .name = "ocrb",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR5B); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR5B = result;
            }
        }
    },
    {
        .name = "ocrc",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendUint16(PASS_FILE_DESCRIPTOR_ARGS, data, OCR5C); },
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) { 
            uint16_t result = 0;
            if (retrieveUint16(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) {
                OCR5C = result;
            }
        }
    },
};

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
struct ush_node_object timer1Dir;
struct ush_node_object timer3Dir;
struct ush_node_object timer4Dir;
struct ush_node_object timer5Dir;



void 
setup() {
    Serial.begin(9600);
    Serial.println("Interface up");
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
#undef NELEM
}

void 
loop() {
    ush_service(&ush);
}



#undef FILE_DESCRIPTOR_ARGS

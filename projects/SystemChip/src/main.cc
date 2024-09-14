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
#include <SD.h>
#include <TimeLib.h>
#include <Wire.h>
#include <SPI.h>
#include <Entropy.h>
#include <EEPROM.h>
#include <IntervalTimer.h>
#include <Bounce.h>
#include <Ethernet.h>
#include <Deception.h>
#include <microshell.h>

#define CACHE_MEMORY_SECTION DMAMEM
#define MEMORY_POOL_SECTION EXTMEM
// fake smi interface
constexpr auto SD0 = 0;
constexpr auto SD1 = 1;
constexpr auto SD2 = 2;
constexpr auto SD3 = 3;
constexpr auto SD4 = 4;
constexpr auto SD5 = 5;
constexpr auto SD6 = 6;
constexpr auto SD7 = 7;
constexpr auto READY_SYNC = 8;
constexpr auto READY = 9;
constexpr auto ADS = 10;
constexpr auto BLAST = 11;
constexpr auto BE0 = 12;
constexpr auto BE1 = 14;
constexpr auto Free0 = 15;
constexpr auto Free1 = 16;
constexpr auto WriteEnable = 25;
constexpr auto OutputEnable = 26;
constexpr auto SA0 = 27;
constexpr auto SA1 = 28;
constexpr auto SA2 = 29;
constexpr auto SA3 = 30;
constexpr auto SA4 = 31;
constexpr auto SA5 = 32;
using Address = uint32_t;
using RawCacheLineData = uint8_t*;
constexpr unsigned long long int operator ""_KB(unsigned long long int value) noexcept {
    return value * 1024;
}
constexpr unsigned long long int operator ""_MB(unsigned long long int value) noexcept {
    return value * 1024 * 1024;
}
constexpr size_t MaxMemoryPoolSize = 16_MB; // PSRAM Pool Maximum Size
constexpr size_t MinimumPoolSize = 1_MB; // we don't want this to go any smaller than this
constexpr auto MemoryPoolSize = 16_MB; 
MEMORY_POOL_SECTION uint8_t memory960[MemoryPoolSize];
static_assert(MemoryPoolSize <= MaxMemoryPoolSize, "Requested memory capacity is too large!");
static_assert(MemoryPoolSize >= MinimumPoolSize, "Requested memory capacity will not fit a default boot image!");

union [[gnu::packed]] DataInterfaceInput {
    struct {
        uint8_t d0 : 1;
        uint8_t d1 : 1;
        uint8_t d2 : 1;
        uint8_t d3 : 1;
        uint8_t d4 : 1;
        uint8_t d5 : 1;
        uint8_t d6 : 1;
        uint8_t d7 : 1;
    };
    uint8_t full;
};
template<typename T>
union [[gnu::packed]] SplitWord {
    T full;
    static constexpr auto NumberOfBytes = sizeof(T)/sizeof(uint8_t);
    uint8_t bytes[NumberOfBytes];
    static_assert(NumberOfBytes < 32);
};
using SplitWord32 = SplitWord<uint32_t>;
using SplitWord16 = SplitWord<uint16_t>;
void setupMicroshell();
void configureParallelInterface() noexcept;
void setDataLines(uint8_t value) noexcept;
uint8_t getDataLines() noexcept;
void setAddress(uint8_t address) noexcept;
void startReadOperation() noexcept;
void endReadOperation() noexcept;
void startWriteOperation() noexcept;
void endWriteOperation() noexcept;
template<decltype(OUTPUT) direction>
inline void configureDataLinesDirection() noexcept {
    pinMode(SD0, direction);
    pinMode(SD1, direction);
    pinMode(SD2, direction);
    pinMode(SD3, direction);
    pinMode(SD4, direction);
    pinMode(SD5, direction);
    pinMode(SD6, direction);
    pinMode(SD7, direction);
}

uint8_t externalBusRead8(uint8_t address) noexcept;
void externalBusWrite8(uint8_t address, uint8_t value) noexcept;
template<typename T>
inline T externalBusRead(uint8_t baseAddress) noexcept {
    using K = SplitWord<T>;
    K tmp;
    for (uint8_t i = 0; i < K::NumberOfBytes; ++i) {
        tmp.bytes[i] = externalBusRead8(baseAddress + i);
    }
    return tmp.full;
}
template<typename T>
inline void externalBusWrite(uint8_t baseAddress, T value) noexcept {
    using K = SplitWord<T>;
    K tmp;
    tmp.full = value;
    for (uint8_t i = 0; i < K::NumberOfBytes; ++i) {
        externalBusWrite8(baseAddress + i, tmp.bytes[i]);
    }
}
void 
setupMemoryPool() {
    // clear out the actual memory pool ahead of setting up the memory pool
    // itself
    memset(memory960, 0, MemoryPoolSize);
}
void
setupSDCard() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("No SDCard found!");
    } else {
        Serial.println("Found an SDCard, will try to transfer the contents of prog.bin to onboard psram");
        auto f = SD.open("prog.bin", FILE_READ); 
        if (!f) {
            Serial.println("Could not open prog.bin...skipping!");
        } else {
            Serial.println("Found prog.bin...");
            if (f.size() <= 0x100000) {
                Serial.println("Transferring prog.bin to memory");
                auto result = f.read(memory960, f.size());
                if (result != f.size()) {
                    Serial.println("prog.bin could not be fully transferred!");
                } else {
                    Serial.println("Transfer complete!");
                    Serial.println("Header Contents:");
                    auto* header = reinterpret_cast<uint32_t*>(memory960);
                    for (int i = 0; i < 8; ++i) {
                        Serial.print("\t0x");
                        Serial.print(i, HEX);
                        Serial.print(": 0x");
                        Serial.println(header[i], HEX);
                    }
                }
            } else {
                Serial.println("prog.bin is too large to fit in 16 megabytes!");
            }
            f.close();
        }
    }
}
volatile bool _systemBooted = false;
void setupServers();
void stateChange2560();
struct [[gnu::packed]] Packet {
    uint8_t typeCode;
    uint32_t address;
    uint8_t size;
    uint8_t data[];
};

void
setupHardware() {
#define X(item, baud, wait) item . begin (baud ) ; \
    if constexpr (wait) { \
        while (! item ) { \
            delay(10) ; \
        } \
    }
    X(Serial, 9600, true);
#undef X
    Serial.println("Interface up");
    configureParallelInterface();
    setupMemoryPool();
    // the sdcard should come last to make sure that we don't clear out all of
    // our work!
    setupSDCard();
    // servers should be setup last to prevent race conditions
    setupServers();
}
void 
setup() {
    _systemBooted = false;
    setupHardware();
    _systemBooted = true;
}

void handleReceiveTop(int howMany);

void handleRequestTop();
class TwoWireServer {
    public:
        TwoWireServer(TwoWire& link) : _link(link) { }
        void begin(uint8_t index, uint32_t clockRate = Deception::TWI_ClockRate) {
            _systemAddress = index;
            _link.begin(index);
            _clockRate = clockRate;
            _link.setClock(clockRate);
            _link.onReceive(handleReceiveTop);
            _link.onRequest(handleRequestTop);
        }
        void handleReceive(int howMany);
        void handleRequest();
        void process();
        [[nodiscard]] constexpr auto getNumberOfMemoryRequests() const noexcept { return _numRequests; }
        [[nodiscard]] constexpr auto getNumberOfMemoryReceives() const noexcept { return _numReceives; }
        [[nodiscard]] constexpr auto getAddress() const noexcept { return _systemAddress; }
        [[nodiscard]] constexpr auto getClockRate() const noexcept { return _clockRate; }
    private:
        void sink();
        void setAddressRegister(uint32_t value) noexcept {
            _address = value;
        }
        void setDataSizeRegister(uint8_t value) noexcept {
            _size = value;
        }
    private:
        TwoWire& _link;
        uint8_t _systemAddress = Deception::TWI_MemoryControllerIndex;
        uint32_t _address = 0x0000'0000;
        uint8_t _size = 0;
        bool _processingRequest = false;
        bool _availableForRead = false;
        uint8_t _capacity = 0;
        uint8_t _index = 0;
        uint64_t _numRequests = 0;
        uint64_t _numReceives = 0;
        uint32_t _clockRate = 0;
        // must be last
        union {
            uint8_t _dataBytes[256] = { 0 };
            Packet op;
        };
};
void
TwoWireServer::handleRequest() {
    ++_numRequests;
    if (_systemBooted) {
        if (_processingRequest) {
            _link.write(static_cast<uint8_t>(Deception::MemoryCodes::CurrentlyProcessingRequest));
        } else {
            if (_availableForRead) {
                if (_size > 0) {
                    _link.write(static_cast<uint8_t>(Deception::MemoryCodes::RequestedData));
                    for (uint32_t a = _address, i = 0; i < _size; ++i, ++a) {
                        _link.write(a < 0x0100'0000 ? memory960[a] : 0);
                    }
                } else {
                    _link.write(static_cast<uint8_t>(Deception::MemoryCodes::NothingToProvide));
                }
            } else {
                _link.write(static_cast<uint8_t>(Deception::MemoryCodes::NothingToProvide));
            }
        }
    } else {
        _link.write(static_cast<uint8_t>(Deception::MemoryCodes::BootingUp));
    }
}

void
TwoWireServer::handleReceive(int howMany) {
    ++_numReceives;
    if (!_processingRequest) {
        if (howMany >= 1) {
            _processingRequest = true;
            _availableForRead = false;
            _capacity = howMany;
            _index = 0;
            while (1 < _link.available()) {
                uint8_t c = _link.read();
                _dataBytes[_index] = c;
                ++_index;
            }
            _dataBytes[_index] = _link.read();
            ++_index;
        }
    }
}

void
TwoWireServer::process() noexcept {
    if (_processingRequest) {
        switch (static_cast<Deception::MemoryCodes>(op.typeCode)) {
            case Deception::MemoryCodes::ReadMemory:
                setAddressRegister(op.address);
                setDataSizeRegister(op.size);
                break;
            case Deception::MemoryCodes::WriteMemory:
                setAddressRegister(op.address);
                setDataSizeRegister(op.size);
                for (uint32_t a = op.address, i = 0; i < op.size; ++a, ++i) {
                    if (a < 0x0100'0000) {
                        memory960[a] = op.data[i];
                    }
                }
                break;
            default:
                break;
        }
        _availableForRead = true;
        _processingRequest = false;
    }
}

void
TwoWireServer::sink() {
    while (_link.available() > 0) {
        (void)_link.read();
    }
}



TwoWireServer link0(Wire);
void 
setupServers() {
    link0.begin(Deception::TWI_MemoryControllerIndex);
    setupMicroshell();
}
void
handleReceiveTop(int howMany) {
    link0.handleReceive(howMany);
}

void
handleRequestTop() {
    link0.handleRequest();
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
    .hostname = "system_chip",
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
        .get_data = timeGetDataCallback,
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

void 
rebootExecCallback(FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
    /// @todo implement reboot at some point
    ush_print(self, "error: reboot not supported");
}
const struct ush_file_descriptor cmdFiles[] = {
    {
        .name = "reboot",
        .description = "reboot device",
        .help = nullptr,
        .exec = rebootExecCallback,
    },
    {
        .name = "memory_stats",
        .description = "display memory connection statistics",
        .help = nullptr,
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            ush_printf(self, "Address: %d\n", link0.getAddress());
            ush_printf(self, "Clock Rate: %ld\n", link0.getClockRate());
            ush_printf(self, "Requests: %lld\n", link0.getNumberOfMemoryRequests());
            ush_printf(self, "Receives: %lld\n", link0.getNumberOfMemoryReceives());
        },
    },
    {
        .name = "read_ioexp",
        .description = "read IO expander values",
        .help = "usage: read_ioexp\n",
        .exec = [](FILE_DESCRIPTOR_ARGS, int argc, char* argv[]) noexcept {
            ush_printf(self, "IOEXP0 : 0x%lx\n", externalBusRead<uint32_t>(0b000'000));
            ush_printf(self, "IOEXP1 : 0x%lx\n", externalBusRead<uint32_t>(0b001'000));
        },
    },

};

const struct ush_file_descriptor memcFiles[] = {
    {
        .name = "address",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char buf[16];
            auto address = link0.getAddress();
            snprintf(buf, sizeof(buf), "%d\n", address);
            buf[sizeof(buf) - 1] = 0;
            *data = (uint8_t*)buf;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "clkrate",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char buf[16];
            auto address = link0.getClockRate();
            snprintf(buf, sizeof(buf), "%ld\n", address);
            buf[sizeof(buf) - 1] = 0;
            *data = (uint8_t*)buf;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "receives",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char buf[32];
            auto value = link0.getNumberOfMemoryReceives();
            snprintf(buf, sizeof(buf), "%lld\n", value);
            buf[sizeof(buf) - 1] = 0;
            *data = (uint8_t*)buf;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "requests",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char buf[32];
            auto value = link0.getNumberOfMemoryRequests();
            snprintf(buf, sizeof(buf), "%lld\n", value);
            buf[sizeof(buf) - 1] = 0;
            *data = (uint8_t*)buf;
            return strlen((char*)(*data));
        },
    },
};

const struct ush_file_descriptor sdCardFiles[] = {
    {
        .name = "present",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            *data = (uint8_t*)((SD.mediaPresent()) ? "1\n" : "0\n");
            // return data size
            return strlen((char*)(*data));
        },
    },
    {
        .name = "capacity",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char buf[32];
            auto value = SD.totalSize();
            snprintf(buf, sizeof(buf), "%lld\n", value);
            buf[sizeof(buf) - 1] = 0;
            *data = (uint8_t*)buf;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "used",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept {
            static char buf[32];
            auto value = SD.usedSize();
            snprintf(buf, sizeof(buf), "%lld\n", value);
            buf[sizeof(buf) - 1] = 0;
            *data = (uint8_t*)buf;
            return strlen((char*)(*data));
        },
    },
};




struct ush_node_object root;
struct ush_node_object dev;
struct ush_node_object bin;
struct ush_node_object cmd;
struct ush_node_object memc;
struct ush_node_object isd;



void
setupMicroshell() {
    ush_init(&ush, &ush_desc);
#define NELEM(obj) (sizeof(obj) / sizeof(obj[0]))
    ush_commands_add(&ush, &cmd, cmdFiles, NELEM(cmdFiles));
    ush_node_mount(&ush, "/", &root, rootFiles, NELEM(rootFiles));
    ush_node_mount(&ush, "/dev", &dev, devFiles, NELEM(devFiles));
    ush_node_mount(&ush, "/bin", &bin, binFiles, NELEM(binFiles));
    ush_node_mount(&ush, "/dev/memc", &memc, memcFiles, NELEM(memcFiles));
    ush_node_mount(&ush, "/dev/sd", &isd, sdCardFiles, NELEM(sdCardFiles));
#undef NELEM
}

void 
loop() {
    link0.process();
    ush_service(&ush);
}

void 
configureParallelInterface() noexcept {
    pinMode(SA0, OUTPUT);
    pinMode(SA1, OUTPUT);
    pinMode(SA2, OUTPUT);
    pinMode(SA3, OUTPUT);
    pinMode(SA4, OUTPUT);
    pinMode(SA5, OUTPUT);
    pinMode(WriteEnable, OUTPUT);
    pinMode(OutputEnable, OUTPUT);
    configureDataLinesDirection<OUTPUT>();
    endWriteOperation();
    endReadOperation();
    setAddress(0);
    externalBusWrite<uint32_t>(0b000100, 0);
    externalBusWrite<uint32_t>(0b001100, 0);
}
void 
setDataLines(uint8_t value) noexcept {
    DataInterfaceInput tmp;
    tmp.full = value;
#define X(index) digitalWrite( SD ## index , tmp.d ## index ) 
    X(0);
    X(1);
    X(2);
    X(3);
    X(4);
    X(5);
    X(6);
    X(7);
#undef X
}
uint8_t 
getDataLines() noexcept {
    DataInterfaceInput tmp;
#define X(index) tmp. d ## index = digitalRead( SD ## index )
    X(0);
    X(1);
    X(2);
    X(3);
    X(4);
    X(5);
    X(6);
    X(7);
#undef X
    return tmp.full;
}

void
setAddress(uint8_t address) noexcept {
    DataInterfaceInput tmp;
    tmp.full = address;
#define X(index) digitalWrite( SA ## index , tmp. d ## index )
    X(0);
    X(1);
    X(2);
    X(3);
    X(4);
    X(5);
#undef X
}

void 
startReadOperation() noexcept {
    digitalWrite(OutputEnable, LOW);
}
void 
endReadOperation() noexcept {
    digitalWrite(OutputEnable, HIGH);
}
void 
startWriteOperation() noexcept {
    digitalWrite(WriteEnable, LOW);
}
void 
endWriteOperation() noexcept {
    digitalWrite(WriteEnable, HIGH);
}

uint8_t 
externalBusRead8(uint8_t address) noexcept {
    configureDataLinesDirection<INPUT>();
    setAddress(address);
    startReadOperation();
    // give it time to respond!
    delayNanoseconds(200);
    auto result = getDataLines();
    endReadOperation();
    return result;
}

void
externalBusWrite8(uint8_t address, uint8_t value) noexcept {
    configureDataLinesDirection<OUTPUT>();
    setAddress(address);
    setDataLines(value);
    startWriteOperation();
    delayNanoseconds(200);
    endWriteOperation();
}



#undef FILE_DESCRIPTOR_ARGS

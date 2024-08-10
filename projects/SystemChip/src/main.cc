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

#define CACHE_MEMORY_SECTION DMAMEM
#define MEMORY_POOL_SECTION EXTMEM
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
CACHE_MEMORY_SECTION Deception::DirectMappedCache_CacheLine16<0x1000> externalCache;
bool sdcardInstalled = false;
static_assert(MemoryPoolSize <= MaxMemoryPoolSize, "Requested memory capacity is too large!");
static_assert(MemoryPoolSize >= MinimumPoolSize, "Requested memory capacity will not fit a default boot image!");
void
setupCaches() {
    externalCache.begin();
}
void 
setupMemoryPool() {
    // clear out the actual memory pool ahead of setting up the memory pool
    // itself
    memset(memory960, 0, MemoryPoolSize);
}
void
setupSDCard() {
    sdcardInstalled = SD.begin(BUILTIN_SDCARD);
    if (!sdcardInstalled) {
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
void setupServers();
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
    setupMemoryPool();
    setupCaches();
    // the sdcard should come last to make sure that we don't clear out all of
    // our work!
    setupSDCard();
    // servers should be setup last to prevent race conditions
    setupServers();
}
void 
setup() {
    setupHardware();
}

void 
loop() {
    
}
struct [[gnu::packed]] Packet {
    uint8_t typeCode;
    uint32_t address;
    uint8_t size;
    uint8_t data[];
};
class HardwareSerialServer {
    public:
        HardwareSerialServer(HardwareSerial& link) : _link(link) { }
        auto& getBackingStore() noexcept { return _link; }
        void begin(uint32_t baud) noexcept { 
            _link.begin(baud); 
            Serial.println("Server Link Up!");
            Serial.println("Waiting for first contact!");
        }
        void handleReadRequest(const Packet& packet) noexcept {
            Serial.print("READ REQUEST 0x");
            Serial.println(packet.address, HEX);
            for (uint32_t a = packet.address, i = 0; i < packet.size; ++i, ++a) {
                _link.write(a < 0x0100'0000 ? memory960[a] : 0);
            }
        }
        void handleWriteRequest(const Packet& packet) noexcept {
            Serial.print("WRITE REQUEST 0x");
            Serial.println(packet.address, HEX);
            for (uint32_t a = packet.address, i = 0; i < packet.size; ++i, ++a) {
                if (a < 0x0100'0000) {
                    memory960[a] = packet.data[i];
                }
            }
        }
        void processPacket() noexcept {
            Packet& packet = *reinterpret_cast<Packet*>(_data);
            Serial.println("PROCESSING PACKET");
            Serial.print("\tTYPECODE: 0x");
            Serial.println(packet.typeCode, HEX);

            switch (packet.typeCode) {
                case Deception::MemoryCodes::ReadMemoryCode:
                    handleReadRequest(packet);
                    break;
                case Deception::MemoryCodes::WriteMemoryCode:
                    handleWriteRequest(packet);
                    break;
                default:
                    break;
            }
        }
        void handleFirstContact() noexcept {
            _firstContact = true;
            _link.write(Deception::MemoryCodes::InitializeSystemSetupCode);
            Serial.println("FIRST CONTACT MADE!");
        }
        void processEvent() noexcept {
            if (auto inByte = _link.read(); !_firstContact) {
                if (inByte == Deception::MemoryCodes::InitializeSystemSetupCode) {
                    handleFirstContact();
                }
            } else {
                if (_serialCapacity == 0) {
                    switch (inByte) {
                        case Deception::MemoryCodes::BeginInstructionCode:
                            Serial.println("New Instruction!");
                            _serialCapacity = -1;
                            _serialCount = 0;
                            break;
                        case Deception::MemoryCodes::InitializeSystemSetupCode:
                            handleFirstContact();
                            break;
                        default:
                            break;
                    }
                } else if (_serialCapacity == -1) {
                    Serial.print("Length: 0x");
                    Serial.println(inByte, HEX);
                    _serialCapacity = inByte;
                } else {
                   _data[_serialCount] = inByte;
                   ++_serialCount;
                   if (_serialCount == _serialCapacity) {
                        processPacket();
                        _serialCapacity = 0;
                        _serialCount = 0;
                   }
                }
            }
        }
    private:
        HardwareSerial& _link;
        bool _firstContact = false;
        int _serialCapacity = 0;
        int _serialCount = 0;
        uint8_t _data[256] = { 0 };
};
HardwareSerialServer link0(Serial8);
void 
setupServers() {
    link0.begin(115200);
}
void 
serialEvent8() {
    link0.processEvent();
}

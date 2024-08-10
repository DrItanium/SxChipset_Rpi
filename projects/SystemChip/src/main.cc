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
static_assert(MemoryPoolSize <= MaxMemoryPoolSize, "Requested memory capacity is too large!");
static_assert(MemoryPoolSize >= MinimumPoolSize, "Requested memory capacity will not fit a default boot image!");
bool sdcardInstalled = false;
void
setupRandomNumberGeneration() {
    uint32_t randomSeedValue = analogRead(A0) + analogRead(A1) + 
        analogRead(A2) + analogRead(A3) + 
        analogRead(A4) + analogRead(A5) +
        analogRead(A6) + analogRead(A7);
    randomSeed(randomSeedValue);
}
void
establishContact() {
}
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
    sdcardInstalled = SD.begin();
    Serial.print("SDCARD ");
    if (!sdcardInstalled) {
        Serial.print("NOT ");
    } 
    Serial.println("FOUND");
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
    setupServers();
    setupSDCard();
    setupMemoryPool();
    setupCaches();
}
void 
setup() {
    setupRandomNumberGeneration();
    setupHardware();
    //establishContact();
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
            for (uint32_t a = packet.address, i = 0; i < packet.size; ++i, ++a) {
                _link.write(a < 0x0100'0000 ? memory960[a] : 0);
            }
        }
        void handleWriteRequest(const Packet& packet) noexcept {
            for (uint32_t a = packet.address, i = 0; i < packet.size; ++i, ++a) {
                if (a < 0x0100'0000) {
                    memory960[a] = packet.data[i];
                }
            }
        }
        void processPacket() noexcept {
            Packet& packet = *reinterpret_cast<Packet*>(_data);
            switch (packet.typeCode) {
                case Deception::MemoryCodes::ReadMemoryCode:
                    handleReadRequest(packet);
                    break;
                case Deception::MemoryCodes::WriteMemoryCode:
                    break;
                default:
                    break;
            }
        }
        void handleFirstContact() noexcept {
            clearInput();
            _firstContact = true;
            _link.write(Deception::MemoryCodes::InitializeSystemSetupCode);
            Serial.println("First Contact Successful!");
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
        void clearInput() noexcept {
            while (_link.available() > 0) {
                (void)_link.read();
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

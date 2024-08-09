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

#define CACHE_MEMORY_SECTION DMAMEM
#define MEMORY_POOL_SECTION EXTMEM
#define DebugPort Serial
#define PCLink Serial4
#define SodiumLink0 Serial7
#define SodiumLink1 Serial8
using Address = uint32_t;
using RawCacheLineData = uint8_t*;
void writeCacheLine(Address address, uint8_t* data, uint8_t size) noexcept;
void readCacheLine(Address address, uint8_t* data, uint8_t size) noexcept;
struct CacheLine {
    static constexpr uint8_t NumDataElements = 16;
    static constexpr uint8_t DataMask = 0xF;
    static constexpr Address AddressMask = 0xFFFF'FFF0;
private:
    Address _key = 0;
    bool _valid = false;
    bool _dirty = false;
    uint8_t _data[NumDataElements] = { 0 };
public:
    constexpr auto getAddress() const noexcept { return _key; }
    constexpr auto isDirty() const noexcept { return _dirty; }
    constexpr auto isValid() const noexcept { return _valid; }
    constexpr auto getData(uint8_t index) const noexcept { return _data[index & DataMask]; }
    void markDirty() noexcept { _dirty = true; }
    void setData(uint8_t index, uint8_t value) noexcept {
        markDirty();
        _data[index & DataMask] = value;
    }
    void sync() noexcept {
        if (_valid) {
            if (_dirty) {
                writeCacheLine(_key, _data, NumDataElements);
                _dirty = false;
            }
        }
    }
    void clear() noexcept {
        _key = 0;
        _valid = false;
        _dirty = false;
        for (int i = 0; i < NumDataElements; ++i) {
            _data[i] = 0;
        }
    }
    static constexpr auto maskAddress(Address targetAddress) noexcept {
        return targetAddress & AddressMask;
    }
    constexpr auto matches(Address targetAddress) const noexcept {
        return _valid & (_key == maskAddress(targetAddress));
    }
    void reuse(Address newKey) {
        sync();
        _valid = true;
        _dirty = false;
        _key = maskAddress(newKey);
        readCacheLine(_key, _data, NumDataElements);
    }
};
void
setupRandomNumberGeneration() {
    uint32_t randomSeedValue = analogRead(A0) + analogRead(A1) + 
        analogRead(A2) + analogRead(A3) + 
        analogRead(A4) + analogRead(A5) +
        analogRead(A6) + analogRead(A7);
    randomSeed(randomSeedValue);
}
constexpr uint8_t InitializeSystemSetupCode = 0xFE;
constexpr uint8_t BeginInstructionCode = 0xFF;
bool sdcardInstalled = false;
void
establishContact() {
    while (PCLink.available() <= 0) {
        PCLink.write(InitializeSystemSetupCode);
        delay(300);
    }
    // clear the serial cache
    (void)PCLink.read();
}
void
setupHardware() {
#define X(item, baud) item . begin (baud ) ; \
    while (! item ) { \
        delay(10) ; \
    }
    X(DebugPort, 9600);
    X(PCLink, 500000);
    X(SodiumLink0, 500000);
    X(SodiumLink1, 500000);
#undef X
    sdcardInstalled = SD.begin();
    DebugPort.print("SDCARD ");
    if (!sdcardInstalled) {
        DebugPort.print("NOT ");
    } 
    DebugPort.println("FOUND");
}
void
setupCaches() {

}
void 
setup() {
    setupRandomNumberGeneration();
    setupHardware();
    setupCaches();
    establishContact();
}

void 
loop() {
    
}


void 
writeCacheLine(Address address, uint8_t* data, uint8_t size) noexcept {

}
void 
readCacheLine(Address address, uint8_t* data, uint8_t size) noexcept {

}

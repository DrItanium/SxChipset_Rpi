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

//#define CACHE_MEMORY_SECTION DMAMEM
#define MEMORY_POOL_SECTION EXTMEM
#define DebugPort Serial
#define PCLink Serial4
#define SodiumLink0 Serial7
#define SodiumLink1 Serial8
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
    while (PCLink.available() <= 0) {
        PCLink.write(InitializeSystemSetupCode);
        delay(300);
    }
    // clear the serial cache
    (void)PCLink.read();
}
void
setupCaches() {
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
    DebugPort.print("SDCARD ");
    if (!sdcardInstalled) {
        DebugPort.print("NOT ");
    } 
    DebugPort.println("FOUND");
}
void
setupHardware() {
#define X(item, baud, wait) item . begin (baud ) ; \
    if constexpr (wait) { \
        while (! item ) { \
            delay(10) ; \
        } \
    }
    X(DebugPort, 9600, false);
    X(PCLink, 500000, false);
    X(SodiumLink0, 115200, false);
    X(SodiumLink1, 115200, false);
#undef X
    setupSDCard();
    setupMemoryPool();
    setupCaches();
}

void 
setup() {
    setupRandomNumberGeneration();
    setupHardware();
    establishContact();
}

void 
loop() {
    
}




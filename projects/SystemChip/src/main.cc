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
        DebugPort.println("NOT ");
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

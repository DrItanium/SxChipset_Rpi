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
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>


#include "Detect.h"
#include "Types.h"
#include "Pinout.h"
#include "Setup.h"

constexpr bool ActivateSDCard = false;
constexpr int32_t SDCardInitializationAttempts = 1000;
volatile bool sdcardAvailable = false;
SdFs SD;

void
trySetupSDCard() noexcept {
    if constexpr (ActivateSDCard) {
        auto initsd = []() -> bool { return SD.begin(static_cast<int>(Pin::SD_EN)); };
        Serial.println(F("Looking for SDCard"));
        if constexpr (SDCardInitializationAttempts < 0) {
            // infinite
            while (!initsd()) {
                Serial.print('.');
                delay(1000);
            }
            Serial.println(F("available!"));
            sdcardAvailable = true;
        } else {
            for (int i = 0; i < SDCardInitializationAttempts; ++i) {
                if (initsd()) {
                    sdcardAvailable = true;
                    break;
                }
                delay(1000); // wait one second
            }
            if (sdcardAvailable) {
                Serial.println(F("SDCard available!"));
            } else {
                Serial.println(F("Max attempts reached, SDCard not available!"));
            }

        }
    }
}
void
reconfigureRandomSeed() noexcept {
    Serial.println(F("Reconfiguring random seed"));
    uint32_t newSeed = 0;
    newSeed += analogRead(A0);
    newSeed += analogRead(A1);
    newSeed += analogRead(A2);
    newSeed += analogRead(A3);
    newSeed += analogRead(A4);
    newSeed += analogRead(A5);
    newSeed += analogRead(A6);
    newSeed += analogRead(A7);
    newSeed += analogRead(A8);
    newSeed += analogRead(A9);
    newSeed += analogRead(A10);
    newSeed += analogRead(A11);
    newSeed += analogRead(A12);
    newSeed += analogRead(A13);
    newSeed += analogRead(A14);
    newSeed += analogRead(A15);
    randomSeed(newSeed);
}
void
configureEBI() noexcept {
    // for now enable a simple 256 byte space
    XMCRB = 0b0'0000'111; // no high bits, no bus keeper
    XMCRA = 0b1'100'00'00; // Enable the EBI, divide the memory space in half,
                           // no wait states in either
}
void
setup() {
    Serial.begin(115200);
    reconfigureRandomSeed();
    Wire.begin();
    SPI.begin();
    trySetupSDCard();
    configureEBI();
}
void 
loop() {
}


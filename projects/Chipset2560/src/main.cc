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

static inline constexpr bool ActivateSDCard = false;
static inline constexpr int32_t SDCardInitializationAttempts = 1000;
SdFs SD;
volatile bool _sdAvailable = false;
volatile i960Interface interface960 [[gnu::address(0x7F00)]];
// With the way that the 2560 and CH351s are connected to the i960, I have to
// transfer data through the 2560 to the i960. This is due to the fact that the
// CH351s are not buffered to prevent this. However, there is nothing stopping
// me from expanding the width of the EBI for my own internal purposes. I can
// also make a 256 byte window into the i960 bus as well.
//
// Actually, if I keep the data lines on the CH351 off the bus (via making them
// inputs) then I can actually do this. I just need to move the data lines to
// buffered parts using two AHC374s per 8-bit block. That way, I can actually
// keep the parts off of the bus as well. 
//
// However, I can easily use the EBI on the 2560 to provide its own internal
// bus. For example, attaching an internal 32k SRAM to allow for more memory
// (if desired). 

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
            _sdAvailable = true;
        } else {
            for (int32_t i = 0; i < SDCardInitializationAttempts; ++i) {
                if (initsd()) {
                    _sdAvailable = true;
                    break;
                }
                delay(1000); // wait one second
            }
            if (_sdAvailable) {
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
    XMCRA = 0b1'100'01'01; // Enable the EBI, divide the memory space in half,
                           // wait states are necessary too
}
[[gnu::always_inline]] inline void clearADSInterrupt() noexcept { bitSet(EIFR, INTF7); }
[[gnu::always_inline]] inline void clearREADYInterrupt() noexcept { bitSet(EIFR, INTF6); }
[[gnu::always_inline]] inline void clearBLASTInterrupt() noexcept { bitSet(EIFR, INTF5); }
[[gnu::always_inline]] inline void clearHLDAInterrupt() noexcept { bitSet(EIFR, INTF4); }

[[gnu::always_inline]] inline void waitForTransaction() noexcept {
    // clear the READY signal interrupt ahead of waiting for the last
    clearREADYInterrupt();
    do{
    } while (bit_is_clear(EIFR, INTF7));
    clearADSInterrupt();
}

[[gnu::always_inline]] inline void waitForReadySynchronization() noexcept {
    do { } while (bit_is_clear(EIFR, INTF6));
    clearREADYInterrupt();
}

//template<uint8_t delayAmount = 6>
void signalReady() noexcept {
    toggle<Pin::READY>();
#if 0
    if constexpr (delayAmount > 0) {
        insertCustomNopCount<delayAmount>();
    }
#else
    waitForReadySynchronization();
#endif
}
void 
configurePins() noexcept {
    pinMode(Pin::READY, OUTPUT);
    digitalWrite<Pin::READY, HIGH>();

    // configure ADS to trigger on rising edge (the address operation is
    // complete)
    pinMode(Pin::ADS, INPUT);
    pinMode(Pin::READY_SYNC_IN, INPUT);
    pinMode(Pin::BLAST, INPUT);
    pinMode(Pin::HLDA, INPUT);
    // Configure INT7 on rising edge of ADS
    // Configure INT6 on rising edge of READY
    // Configure INT5 on falling edge of BLAST
    // Configure INT4 on rising edge of HLDA (the hold request is acknowledged)
    EICRB = 0b11'11'10'11;
    // clear the interrupt flags to be on the safe side
    bitSet(EIFR, INTF4);
    bitSet(EIFR, INTF5);
    bitSet(EIFR, INTF6);
    bitSet(EIFR, INTF7);
}

void
setup() {
    configureEBI();
    interface960.begin();
    reconfigureRandomSeed();
    configurePins();
    Serial.begin(115200);
    Wire.begin();
    SPI.begin();
    trySetupSDCard();
    
    delay(1000);
    interface960.pullI960OutOfReset();
}
[[gnu::always_inline]] inline bool isReadOperation() noexcept {
    return interface960.isReadOperation();
}
[[gnu::always_inline]] inline bool isLastWordOfTransaction() noexcept {
    return interface960.lastWordOfTransaction();
}
void
doMemoryWriteTransaction(uint32_t address) noexcept {
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();

        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    // last part of the word will just be finished soon
    clearBLASTInterrupt();
    signalReady();

}
void
doIOWriteTransaction(uint32_t address) noexcept {
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    // last part of the word will just be finished soon
    signalReady();
}
void
doMemoryReadTransaction(uint32_t address) noexcept {
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    // last part of the word will just be finished soon
    signalReady();
}
template<bool isReadOperation>
void
doNothingOperation() noexcept {
    if constexpr(isReadOperation) {
        interface960.dataLines.full = 0;
    }
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    // last part of the word will just be finished soon
    signalReady();
}
void
doIOReadTransaction(uint32_t address) noexcept {
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    // last part of the word will just be finished soon
    signalReady();
}
[[gnu::always_inline]] inline bool isIOOperation(uint32_t address) {
    return (static_cast<uint8_t>(address >> 24)) == 0xFE;
}
void 
doReadTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0xFFFF;
    if (isIOOperation(address)) {
        doIOReadTransaction(address);
    } else {
        doMemoryReadTransaction(address);
    }
}
void
doWriteTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0;
    if (isIOOperation(address)) {
        doIOWriteTransaction(address);
    } else {
        doMemoryWriteTransaction(address);
    }
}

template<bool isReadOperation>
void
doTransaction(uint32_t address) noexcept {
    if constexpr (isReadOperation) {
        doReadTransaction(address);
    } else {
        doWriteTransaction(address);
    }
}
void 
loop() {
    waitForTransaction();
    auto address = interface960.getAddress();
    Serial.printf(F("address lines: 0x%lx\n"), address);
    if (isReadOperation()) {
        doTransaction<true>(address);
    } else {
        doTransaction<false>(address);
    }
}

void 
i960Interface::begin() volatile {
    addressDirection = 0x0000'0000; 
    dataLinesDirection = 0xFFFF;
    // turn ready into an input
    controlDirection = 0b0001'1111'1000'0000;
    controlLines.int3 = 1;
    controlLines.int2 = 0;
    controlLines.int1 = 0;
    controlLines.int0 = 1;
    controlLines.hold = 0;
    controlLines.reset = 0;
    dataLines.full = 0;
}

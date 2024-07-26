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
configureEBI() noexcept {
    // for now enable a simple 256 byte space
    XMCRB = 0b0'0000'111; // no high bits, no bus keeper
    XMCRA = 0b1'100'01'01; // Enable the EBI, divide the memory space in half,
                           // wait states are necessary too
}
#define ADSFLAG INTF7
#define READYFLAG INTF6
#define BLASTFLAG INTF5
#define HLDAFLAG INTF4
[[gnu::always_inline]] inline void clearADSInterrupt() noexcept { bitSet(EIFR, ADSFLAG); }
[[gnu::always_inline]] inline void clearREADYInterrupt() noexcept { bitSet(EIFR, READYFLAG); }
[[gnu::always_inline]] inline void clearBLASTInterrupt() noexcept { bitSet(EIFR, BLASTFLAG); }
[[gnu::always_inline]] inline void clearHLDAInterrupt() noexcept { bitSet(EIFR, HLDAFLAG); }

[[gnu::always_inline]] inline void waitForTransaction() noexcept {
    // clear the READY signal interrupt ahead of waiting for the last
    clearREADYInterrupt();
    do{ } while (bit_is_clear(EIFR, ADSFLAG));
    clearADSInterrupt();
}



[[gnu::always_inline]] inline void signalReady() noexcept {
    toggle<Pin::READY>();
    do { } while (bit_is_clear(EIFR, READYFLAG));
    clearREADYInterrupt();
}
void
configureInterrupts() noexcept {
    // Configure INT7 on rising edge of ADS
    // Configure INT6 on rising edge of READY
    // Configure INT5 on falling edge of BLAST
    // Configure INT4 on rising edge of HLDA (the hold request is acknowledged)
    EICRB = 0b11'11'10'11;
    // clear the interrupt flags to be on the safe side
    clearHLDAInterrupt();
    clearBLASTInterrupt();
    clearREADYInterrupt();
    clearADSInterrupt();

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
}

void
setup() {
    configureEBI();
    interface960.begin();
    configurePins();
    configureInterrupts();
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
template<bool useIOExpander = false>
[[gnu::always_inline]] inline bool isLastWordOfTransaction() noexcept {
    if constexpr (useIOExpander) {
        return interface960.lastWordOfTransaction();
    } else {
        return bit_is_set(EIFR, BLASTFLAG);
    }
}
template<bool isReadOperation>
void
doNothingOperation() noexcept {
    if constexpr(isReadOperation) {
        interface960.dataLines.full = 0;
    }
    while (!isLastWordOfTransaction()) {
        signalReady();
    }
    clearBLASTInterrupt();
    signalReady();
}
void 
genericExecutionBody() noexcept {
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
}
[[gnu::always_inline]] inline bool isIOOperation(uint32_t address) {
    return (static_cast<uint8_t>(address >> 24)) == 0xFE;
}
void 
doReadTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0xFFFF;
#if 0
    if (isIOOperation(address)) {
        doIOReadTransaction(address);
    } else {
        doMemoryReadTransaction(address);
    }
#else 
    doNothingOperation<true>();
#endif
}
void
doWriteTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0;
#if 0
    if (isIOOperation(address)) {
        doIOWriteTransaction(address);
    } else {
        doMemoryWriteTransaction(address);
    }
#else
    doNothingOperation<false>();
#endif

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

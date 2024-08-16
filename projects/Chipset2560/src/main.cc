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
#include <Wire.h>
#include <Deception.h>


//#include "Detect.h"
#include "Types.h"
#include "Pinout.h"
#include "Setup.h"

Deception::HardwareSerialBackingStore PCLink(Serial1);
Deception::DirectMappedCache4K onboardCache;
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
//


// the memory that we are actually getting data from comes from the PCLink
// serial device that we cache on chip (eventually over the EBI as well to
// increase the availability of data as well)

#define ADSFLAG INTF7
#define READYFLAG INTF6
#define BLASTFLAG INTF5
#define HLDAFLAG INTF4
[[gnu::always_inline]] inline void clearADSInterrupt() noexcept { bitSet(EIFR, ADSFLAG); }
[[gnu::always_inline]] inline void clearREADYInterrupt() noexcept { bitSet(EIFR, READYFLAG); }
[[gnu::always_inline]] inline void clearBLASTInterrupt() noexcept { bitSet(EIFR, BLASTFLAG); }
[[gnu::always_inline]] inline void clearHLDAInterrupt() noexcept { bitSet(EIFR, HLDAFLAG); }

[[gnu::always_inline]] 
inline void 
waitForTransaction() noexcept {
    // clear the READY signal interrupt ahead of waiting for the last
    clearREADYInterrupt();
    do{ } while (bit_is_clear(EIFR, ADSFLAG));
    clearADSInterrupt();
}



[[gnu::always_inline]] 
inline void 
signalReady() noexcept {
    toggle<Pin::READY>();
    do { } while (bit_is_clear(EIFR, READYFLAG));
    clearREADYInterrupt();
}

[[gnu::always_inline]]
[[nodiscard]]
inline bool 
lowerByteEnabled() noexcept {
    return digitalRead<Pin::BE0>() == LOW;
}

[[gnu::always_inline]]
[[nodiscard]]
inline bool 
upperByteEnabled() noexcept {
    return digitalRead<Pin::BE1>() == LOW;
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint8_t 
lowerData() noexcept {
    return getInputRegister<Port::DataLower>();
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint8_t 
upperData() noexcept {
    return getInputRegister<Port::DataUpper>();
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint16_t 
data() noexcept {
    union {
        uint8_t bytes[sizeof(uint16_t)];
        uint16_t full;
    } storage;

    storage.bytes[0] = lowerData();
    storage.bytes[1] = upperData();
    return storage.full;
}
[[gnu::always_inline]]
inline void
setUpperData(uint8_t value) noexcept {
    getOutputRegister<Port::DataUpper>() = value;
}
[[gnu::always_inline]]
inline void
setLowerData(uint8_t value) noexcept {
    getOutputRegister<Port::DataLower>() = value;
}
[[gnu::always_inline]]
inline void
setDataValue(uint16_t value) noexcept {
    setLowerData(value);
    setUpperData(value >> 8);
}
[[gnu::always_inline]]
inline void
setDataDirection(uint16_t value) noexcept {
    getDirectionRegister<Port::DataLower>() = static_cast<uint8_t>(value);
    getDirectionRegister<Port::DataUpper>() = static_cast<uint8_t>(value >> 8);
}

void
setup() {
    pinMode(Pin::RESET, OUTPUT);
    digitalWrite<Pin::RESET, LOW>();
    pinMode(Pin::HOLD , OUTPUT);
    pinMode(Pin::READY, OUTPUT);
    pinMode(Pin::ADS, INPUT);
    pinMode(Pin::READY_SYNC_IN, INPUT);
    pinMode(Pin::BLAST, INPUT);
    pinMode(Pin::HLDA, INPUT);
    pinMode(Pin::READY, OUTPUT);
    pinMode(Pin::BE0, INPUT);
    pinMode(Pin::BE1, INPUT);
    pinMode(Pin::FAIL, INPUT);
    pinMode(Pin::LOCK, INPUT);
    pinMode(Pin::WR, INPUT);
    setDataDirection(0xFFFF);
    getDirectionRegister<Port::AddressLowest>() = 0;
    getDirectionRegister<Port::AddressLower>() = 0;
    getDirectionRegister<Port::AddressHigher>() = 0;
    getDirectionRegister<Port::AddressHighest>() = 0;
    digitalWrite<Pin::HOLD, LOW>();
    digitalWrite<Pin::READY, HIGH>();
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
    Serial.begin(115200);
    Serial1.begin(115200);
    Wire.begin();
    SPI.begin();
    onboardCache.begin();
    PCLink.connect();
    (void)PCLink.getBackingStore().read();
    delay(1000);
    digitalWrite<Pin::RESET, HIGH>();
}
[[gnu::always_inline]] inline bool isReadOperation() noexcept {
    return digitalRead<Pin::WR>() == LOW;
}
[[gnu::always_inline]] inline bool isLastWordOfTransaction() noexcept {
    return bit_is_set(EIFR, BLASTFLAG);
}
template<bool isReadOperation>
void
doNothingOperation() noexcept {
    if constexpr(isReadOperation) {
        setDataValue(0);
    }
    while (!isLastWordOfTransaction()) {
        signalReady();
    }
    clearBLASTInterrupt();
    signalReady();
}
void
send32BitConstant(uint32_t value) noexcept {
    setDataValue(value);
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    setDataValue(value >> 16);
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    doNothingOperation<true>();
}

void
send16BitValue(uint16_t value) noexcept {
    setDataValue(value);
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    doNothingOperation<true>();
}
void
send16BitValue(uint8_t lo, uint8_t hi) noexcept {
    setLowerData(lo);
    setUpperData(hi);
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    doNothingOperation<true>();
}

template<bool readOperation>
void
doIOTransaction(uint32_t address) noexcept {
    switch (address & 0x00FF'FFFF) {
        case 0x0:
            if constexpr (readOperation) {
                send32BitConstant(F_CPU);
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x4:
            if constexpr (readOperation) {
                send32BitConstant(F_CPU/2);
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x8:
            if constexpr (readOperation) {
                send16BitValue(Serial.read());
            } else {
                Serial.write(lowerData());
                doNothingOperation<readOperation>();
            }
            break;
        case 0xC:
            if constexpr (!readOperation) {
                Serial.flush();
            }
            doNothingOperation<readOperation>();
            break;
        case 0x40:
            if constexpr (readOperation)  {
                send32BitConstant(millis());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x44:
            if constexpr (readOperation) {
                send32BitConstant(micros());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        default:
            doNothingOperation<readOperation>();
            break;
    }
}
template<bool readOperation>
void
doMemoryTransaction(uint32_t address) noexcept {
    auto offset = static_cast<uint8_t>(address & 0xF);
    auto& line = onboardCache.find(PCLink, address);
    auto* ptr = line.getLineData(offset);
    if constexpr (!readOperation) {
        line.markDirty();
    }
#define X(base) { \
    if constexpr (readOperation) { \
        setLowerData(ptr[base + 0]); \
        setUpperData(ptr[base + 1]); \
    } else { \
        if (lowerByteEnabled()) { ptr[base + 0] = lowerData(); } \
        if (upperByteEnabled()) { ptr[base + 1] = upperData(); } \
    } \
    if (isLastWordOfTransaction()) { \
        clearBLASTInterrupt(); \
        signalReady(); \
        return; \
    } \
    signalReady(); \
}
    X(0);
    X(2);
    X(4);
    X(6);
    X(8);
    X(10);
    X(12);
    X(14);
#undef X
}
[[gnu::always_inline]] inline bool isIOOperation() noexcept {
    return getInputRegister<Port::AddressHighest>() == 0xFE;
}
void 
doReadTransaction(uint32_t address) noexcept {
    setDataDirection(0xFFFF);
    if (isIOOperation()) {
        doIOTransaction<true>(address);
    } else {
        doMemoryTransaction<true>(address);
    }
}


void
doWriteTransaction(uint32_t address) noexcept {
    setDataDirection(0);
    if (isIOOperation()) {
        doIOTransaction<false>(address);
    } else {
        doMemoryTransaction<false>(address);
    }
}

uint32_t 
getAddress() noexcept {
    union {
        uint8_t bytes[sizeof(uint32_t)];
        uint32_t value;
    } storage;
    storage.bytes[0] = getInputRegister<Port::AddressLowest>();
    storage.bytes[1] = getInputRegister<Port::AddressLower>();
    storage.bytes[2] = getInputRegister<Port::AddressHigher>();
    storage.bytes[3] = getInputRegister<Port::AddressHighest>();
    return storage.value;
}

void 
loop() {
    waitForTransaction();
    auto address = getAddress();
    //Serial.printf(F("address lines: 0x%lx\n"), address);
    if (isReadOperation()) {
        doReadTransaction(address);
    } else {
        doWriteTransaction(address);
    }
}

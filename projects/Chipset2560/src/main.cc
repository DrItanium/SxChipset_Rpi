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


#include "Detect.h"
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

[[gnu::always_inline]] inline void waitForTransaction() noexcept {
    // clear the READY signal interrupt ahead of waiting for the last
    clearREADYInterrupt();
    do{ 
        yield();
    } while (bit_is_clear(EIFR, ADSFLAG));
    clearADSInterrupt();
}



[[gnu::always_inline]] inline void signalReady() noexcept {
    toggle<Pin::READY>();
    do { 
        yield();
    } while (bit_is_clear(EIFR, READYFLAG));
    clearREADYInterrupt();
    Serial.println(F("NEXT"));
}
bool 
lowerByteEnabled() noexcept {
    return digitalRead<Pin::BE0>() == LOW;
}
bool 
upperByteEnabled() noexcept {
    return digitalRead<Pin::BE1>() == LOW;
}
uint8_t 
lowerData() noexcept {
    return getInputRegister<Port::DataLower>();
}
uint8_t 
upperData() noexcept {
    return getInputRegister<Port::DataUpper>();
}
uint16_t 
data() noexcept {
    union {
        uint8_t bytes[sizeof(uint16_t)];
        uint16_t full;
    } storage;

    storage.bytes[0] = lowerData();
    storage.bytes[1] = upperData();
    return storage.full;
}
void
setUpperData(uint8_t value) noexcept {
    getOutputRegister<Port::DataUpper>() = value;
}
void
setLowerData(uint8_t value) noexcept {
    getOutputRegister<Port::DataLower>() = value;
}
void
setDataValue(uint16_t value) noexcept {
    setLowerData(value);
    setUpperData(value >> 8);
}
void
setDataDirection(uint16_t value) noexcept {
    getDirectionRegister<Port::DataLower>() = static_cast<uint8_t>(value);
    getDirectionRegister<Port::DataUpper>() = static_cast<uint8_t>(value >> 8);
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
    getDirectionRegister<Port::DataLower>() = 0xFF;
    getDirectionRegister<Port::DataUpper>() = 0xFF;
    getDirectionRegister<Port::AddressLowest>() = 0;
    getDirectionRegister<Port::AddressLower>() = 0;
    getDirectionRegister<Port::AddressHigher>() = 0;
    getDirectionRegister<Port::AddressHighest>() = 0;
    digitalWrite<Pin::HOLD, LOW>();
    digitalWrite<Pin::READY, HIGH>();

}

void
setup() {
    configurePins();
    Serial.begin(115200);
    Serial1.begin(115200);
    Wire.begin();
    SPI.begin();
    configureInterrupts();
    onboardCache.begin();
    PCLink.connect();
    (void)PCLink.getBackingStore().read();
    delay(1000);
    //interface960.pullI960OutOfReset();
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
void
doIOReadTransaction(uint32_t address) noexcept {
    switch (address & 0x00FFFFFF) {
        case 0x0:
            send32BitConstant(F_CPU);
            break;
        case 0x4:
            send32BitConstant(F_CPU/2);
            break;
        case 0x8:
            send16BitValue(Serial.read());
            break;
        case 0x40:
            send32BitConstant(millis());
            break;
        case 0x44:
            send32BitConstant(micros());
            break;
        /// @todo implement disk operations?
        default:
            doNothingOperation<true>();
            break;
    }
}
void
doIOWriteTransaction(uint32_t address) noexcept {
    switch (address & 0x00FFFFFF) {
        case 0x8: 
            Serial.write(lowerData());
            doNothingOperation<false>();
            break;
        case 0xC:
            Serial.flush();
            doNothingOperation<false>();
            break;
        /// @todo implement disk operations eventually
        default:
            doNothingOperation<false>();
            break;
    }
}
void
doMemoryReadTransaction(uint32_t address) noexcept {
    auto offset = static_cast<uint8_t>(address & 0xF);
    auto& line = onboardCache.find(PCLink, address);
    auto* ptr = line.getLineData(offset);
    {
        setLowerData(ptr[0]);
        setUpperData(ptr[1]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[2]);
        setUpperData(ptr[3]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[4]);
        setUpperData(ptr[5]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[6]);
        setUpperData(ptr[7]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[8]);
        setUpperData(ptr[9]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[10]);
        setUpperData(ptr[11]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[12]);
        setUpperData(ptr[13]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
        setLowerData(ptr[14]);
        setUpperData(ptr[15]);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
}
void 
doMemoryWriteTransaction(uint32_t address) noexcept {
    auto offset = address & 0xF;
    auto& line = onboardCache.find(PCLink, address);
    auto* ptr = line.getLineData(offset);
    line.markDirty();
    if (lowerByteEnabled()) { ptr[0] = lowerData(); }
    if (upperByteEnabled()) { ptr[1] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[2] = lowerData(); }
    if (upperByteEnabled()) { ptr[3] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[4] = lowerData(); }
    if (upperByteEnabled()) { ptr[5] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[6] = lowerData(); }
    if (upperByteEnabled()) { ptr[7] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[8] = lowerData(); }
    if (upperByteEnabled()) { ptr[9] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[10] = lowerData(); }
    if (upperByteEnabled()) { ptr[11] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[12] = lowerData(); }
    if (upperByteEnabled()) { ptr[13] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (lowerByteEnabled()) { ptr[14] = lowerData(); }
    if (upperByteEnabled()) { ptr[15] = upperData(); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
}
[[gnu::always_inline]] inline bool isIOOperation() noexcept {
    return getInputRegister<Port::AddressHighest>() == 0xFE;
}
void 
doReadTransaction(uint32_t address) noexcept {
    setDataDirection(0xFFFF);
    if (isIOOperation()) {
        doIOReadTransaction(address);
    } else {
        doMemoryReadTransaction(address);
    }
}


void
doWriteTransaction(uint32_t address) noexcept {
    setDataDirection(0);
    if (isIOOperation()) {
        doIOWriteTransaction(address);
    } else {
        doMemoryWriteTransaction(address);
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
    //Serial.println(F("Waiting for ADS"));
    waitForTransaction();
    auto address = getAddress();
    //Serial.printf(F("address lines: 0x%lx\n"), address);
    if (isReadOperation()) {
        doReadTransaction(address);
    } else {
        doWriteTransaction(address);
    }
}

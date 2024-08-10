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

Deception::HardwareSerialBackingStore PCLink(Serial2);
Deception::DirectMappedCache4K onboardCache;
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
//


// the memory that we are actually getting data from comes from the PCLink
// serial device that we cache on chip (eventually over the EBI as well to
// increase the availability of data as well)

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
    Serial.begin(115200);
    Serial2.begin(115200);
    configureEBI();
    interface960.begin();
    configurePins();
    configureInterrupts();
    onboardCache.begin();
    PCLink.connect();
    (void)PCLink.getBackingStore().read();
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
send32BitConstant(uint32_t value) noexcept {
    interface960.dataLines.full = static_cast<uint16_t>(value);
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    interface960.dataLines.full = static_cast<uint16_t>(value>> 16);
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
    interface960.dataLines.full = value;
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
    interface960.dataLines.bytes[0] = lo;
    interface960.dataLines.bytes[1] = hi;
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
            Serial.write(interface960.dataLines.bytes[0]);
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
    auto offset = address & 0xF;
    auto& line = onboardCache.find(PCLink, address);
#define X(x, index) interface960.dataLines.bytes[x] = line.getByte(offset + index)
    {
    X(0, 0);
    X(1, 1);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0, 2);
    X(1, 3);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0,4);
    X(1,5);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0,6);
    X(1,7);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0,8);
    X(1,9);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0,10);
    X(1,11);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0,12);
    X(1,13);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    {
    X(0,14);
    X(1,15);
    }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
#undef X
}
void 
doMemoryWriteTransaction(uint32_t address) noexcept {
    auto offset = address & 0xF;
    auto& line = onboardCache.find(PCLink, address);
    if (interface960.controlLines.be0) { line.setByte(offset + 0, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 1, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 2, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 3, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 4, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 5, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 6, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 7, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 8, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 9, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 10, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 11, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 12, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 13, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
    if (interface960.controlLines.be0) { line.setByte(offset + 14, interface960.dataLines.bytes[0]); }
    if (interface960.controlLines.be1) { line.setByte(offset + 15, interface960.dataLines.bytes[1]); }
    if (isLastWordOfTransaction()) {
        clearBLASTInterrupt();
        signalReady();
        return;
    }
    signalReady();
}
[[gnu::always_inline]] inline bool isIOOperation(uint32_t address) noexcept {
    return (static_cast<uint8_t>(address >> 24)) == 0xFE;
}
[[gnu::always_inline]] inline bool isIOOperation() noexcept {
    return interface960.isIOOperation();
}
void 
doReadTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0xFFFF;
    if (interface960.isIOOperation()) {
        doIOReadTransaction(address);
    } else {
        doMemoryReadTransaction(address);
    }
}


void
doWriteTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0;
    if (interface960.isIOOperation()) {
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


void 
loop() {
    //Serial.println(F("Waiting for ADS"));
    waitForTransaction();
    auto address = interface960.getAddress();
    //Serial.printf(F("address lines: 0x%lx\n"), address);
    if (isReadOperation()) {
        doTransaction<true>(address);
    } else {
        doTransaction<false>(address);
    }
}

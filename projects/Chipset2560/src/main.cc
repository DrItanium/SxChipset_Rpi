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

#define PCLink Serial3
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
static inline constexpr bool ActivateSDCard = false;
static inline constexpr int32_t SDCardInitializationAttempts = 1000;
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
    configureEBI();
    interface960.begin();
    configurePins();
    configureInterrupts();
    Serial.begin(115200);
    PCLink.begin(115200);
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
[[gnu::always_inline]] inline bool isIOOperation(uint32_t address) noexcept {
    return (static_cast<uint8_t>(address >> 24)) == 0xFE;
}
[[gnu::always_inline]] inline bool isIOOperation() noexcept {
    return interface960.isIOOperation();
}
void 
doReadTransaction(uint32_t address) noexcept {
    interface960.dataLinesDirection = 0xFFFF;
#if 0
    if (interface960.isIOOperation(address)) {
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
void
sendCommandHeader(uint8_t size, uint8_t code) noexcept {
    PCLink.write(MemoryCodes::BeginInstructionCode);
    PCLink.write(size);
    PCLink.write(code);
}
void
send32BitNumber(uint32_t number) {
    PCLink.write(reinterpret_cast<char*>(&number), sizeof(number));
}

void 
writeCacheLine(Address address, uint8_t* data, uint8_t size) noexcept {
    sendCommandHeader(1 + sizeof(address) + size + 1, MemoryCodes::WriteMemoryCode);
    send32BitNumber(address);
    PCLink.write(size);
    PCLink.write(data, size);
}
void 
readCacheLine(Address address, uint8_t* data, uint8_t size) noexcept {
    sendCommandHeader(1 + sizeof(address) + 1, MemoryCodes::WriteMemoryCode);
    send32BitNumber(address);
    PCLink.write(size);
    PCLink.readBytes(reinterpret_cast<char*>(data), size);
}


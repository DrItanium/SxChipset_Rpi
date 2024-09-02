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
#include <Wire.h>
#include <Deception.h>


#include "Types.h"
#include "Pinout.h"
#include "Setup.h"

constexpr bool EnableStateDebuggingPins = true;
Deception::TwoWireBackingStore PCLink2(Wire, Deception::TWI_MemoryControllerIndex);
using DataCache = Deception::DirectMappedCache<256, Deception::CacheLine16<Deception::TwoWireBackingStore>>;
DataCache onboardCache;
union [[gnu::packed]] SplitWord32 {
    uint8_t bytes[sizeof(uint32_t) / sizeof(uint8_t)];
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    __uint24 lo24;
    uint32_t full;
    constexpr SplitWord32(uint32_t value = 0) : full(value) { }
    constexpr SplitWord32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : bytes{a, b, c, d} { }
    constexpr bool isIOOperation() const noexcept { return bytes[3] == 0xFE; }
    constexpr auto getCacheOffset() const noexcept {
        return DataCache::computeOffset(bytes[0]);
    }
};
static_assert(sizeof(SplitWord32) == sizeof(uint32_t));

#define ADSFLAG INTF4
#define ADS_ISC0 ISC40
#define ADS_ISC1 ISC41
#define BLASTFLAG INTF5
#define BLAST_ISC0 ISC50
#define BLAST_ISC1 ISC51
#define HLDAFLAG INTF6
#define HLDA_ISC0 ISC60
#define HLDA_ISC1 ISC61
#define READYFLAG INTF7
#define READY_ISC0 ISC70
#define READY_ISC1 ISC71
[[gnu::always_inline]] inline void clearADSInterrupt() noexcept { bitSet(EIFR, ADSFLAG); }
[[gnu::always_inline]] inline void clearREADYInterrupt() noexcept { bitSet(EIFR, READYFLAG); }
[[gnu::always_inline]] inline void clearBLASTInterrupt() noexcept { bitSet(EIFR, BLASTFLAG); }
[[gnu::always_inline]] inline void clearHLDAInterrupt() noexcept { bitSet(EIFR, HLDAFLAG); }

void 
configureInterruptSources() noexcept {
    EIMSK = 0x00;
    EICRB = 0x00;
    // Configure rising edge of HLDA (the hold request is acknowledged)
    bitSet(EICRB, HLDA_ISC0);
    bitSet(EICRB, HLDA_ISC1);
    // Configure rising edge of ADS
    bitSet(EICRB, ADS_ISC0);
    bitSet(EICRB, ADS_ISC1);
    // Configure rising edge of READY (falling edge can also be quite safe as
    // well
    //bitSet(EICRB, READY_ISC0);
    bitClear(EICRB, READY_ISC0);
    bitSet(EICRB, READY_ISC1);

    clearHLDAInterrupt();
    clearADSInterrupt();
    clearREADYInterrupt();
}

[[gnu::always_inline]]
inline 
void
waitForReady() noexcept {
    while (bit_is_clear(EIFR, READYFLAG));
    clearREADYInterrupt();
}
template<bool wait = true>
[[gnu::always_inline]] 
inline void 
signalReady() noexcept {
    toggle<Pin::READY>();
    if constexpr (wait) {
        waitForReady();
    }
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

template<uint16_t value>
[[gnu::always_inline]]
inline void
setDataDirection() noexcept {
    getDirectionRegister<Port::DataLower>() = static_cast<uint8_t>(value);
    getDirectionRegister<Port::DataUpper>() = static_cast<uint8_t>(value >> 8);
}

[[gnu::always_inline]]
inline void
configureDataLinesForRead() noexcept {
    setDataDirection<0xFFFF>();
}

[[gnu::always_inline]]
inline void
configureDataLinesForWrite() noexcept {
    setDataDirection<0>();
}

[[gnu::always_inline]] inline bool isReadOperation() noexcept {
    return digitalRead<Pin::WR>() == LOW;
}
[[gnu::always_inline]] inline bool isLastWordOfTransaction() noexcept {
    return digitalRead<Pin::BLAST>() == LOW;
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
    signalReady();
}
void
send32BitConstant(uint32_t value) noexcept {
    setDataValue(value);
    if (isLastWordOfTransaction()) {
        signalReady();
        return;
    }
    signalReady();
    setDataValue(value >> 16);
    if (isLastWordOfTransaction()) {
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
        signalReady();
        return;
    }
    signalReady();
    doNothingOperation<true>();
}

template<bool readOperation>
void
doIOTransaction(SplitWord32 address) noexcept {
    switch (address.lo24) {
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
inline
void
doMemoryTransaction(SplitWord32 address) noexcept {
    if constexpr (EnableStateDebuggingPins) {
        digitalWrite<Pin::CacheLineLookup, LOW>();
    }
    auto& line = onboardCache.find(PCLink2, address.full);
    if constexpr (EnableStateDebuggingPins) {
        digitalWrite<Pin::CacheLineLookup, HIGH>();
    }
    auto* ptr = line.getLineData(address.getCacheOffset());
    if constexpr (!readOperation) {
        line.markDirty();
    }
    if constexpr (readOperation) {
        uint8_t lo = ptr[0];
        uint8_t hi = ptr[1];
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[2];
        hi = ptr[3];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[4];
        hi = ptr[5];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[6];
        hi = ptr[7];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[8];
        hi = ptr[9];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[10];
        hi = ptr[11];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[12];
        hi = ptr[13];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady<false>();
        lo = ptr[14];
        hi = ptr[15];
        waitForReady();
        setLowerData(lo);
        setUpperData(hi);
        signalReady();
    } else {
        if (lowerByteEnabled()) ptr[0] = lowerData();
        if (upperByteEnabled()) ptr[1] = upperData();
        if (isLastWordOfTransaction()) {
            signalReady();
            return;
        }
        signalReady();
        // we can safely ignore checking BE0 since we flowed into this
        auto lo = lowerData();
        auto hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[3] = hi;
            signalReady<false>();
            ptr[2] = lo;
            waitForReady();
            return;
        }
        signalReady<false>();
        ptr[2] = lo;
        ptr[3] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        lo = lowerData();
        hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[5] = hi;
            signalReady<false>();
            ptr[4] = lo;
            waitForReady();
            return;
        }
        signalReady<false>();
        ptr[4] = lo;
        ptr[5] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        lo = lowerData();
        hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[7] = hi;
            signalReady<false>();
            ptr[6] = lo;
            waitForReady();
            return;
        }
        signalReady<false>();
        ptr[6] = lo;
        ptr[7] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        lo = lowerData();
        hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[9] = hi;
            signalReady<false>();
            ptr[8] = lo;
            waitForReady();
            return;
        }
        signalReady<false>();
        ptr[8] = lo;
        ptr[9] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        lo = lowerData();
        hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[11] = hi;
            signalReady<false>();
            ptr[10] = lo;
            waitForReady();
            return;
        }
        signalReady<false>();
        ptr[10] = lo;
        ptr[11] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        lo = lowerData();
        hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[13] = hi;
            signalReady<false>();
            ptr[12] = lo;
            waitForReady();
            return;
        }
        signalReady<false>();
        ptr[12] = lo;
        ptr[13] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        ptr[14] = lowerData();
        if (upperByteEnabled()) ptr[15] = upperData();
        signalReady();
    }
}

inline
SplitWord32
getAddress() noexcept {
    return { 
        getInputRegister<Port::AddressLowest>(),
        getInputRegister<Port::AddressLower>(),
        getInputRegister<Port::AddressHigher>(),
        getInputRegister<Port::AddressHighest>()
    };
}

void
configurePins() noexcept {
    pinMode(Pin::CacheLineLookup, OUTPUT);
    pinMode(Pin::TransactionStatus, OUTPUT);
    pinMode(Pin::ReadyWaitDone, OUTPUT);
    digitalWrite<Pin::ReadyWaitDone, HIGH>();
    digitalWrite<Pin::TransactionStatus, HIGH>();
    digitalWrite<Pin::CacheLineLookup, HIGH>();
    pinMode(Pin::RESET, OUTPUT);
    digitalWrite<Pin::RESET, LOW>();
    pinMode(Pin::INT960_0, OUTPUT);
    pinMode(Pin::INT960_1, OUTPUT);
    pinMode(Pin::INT960_2, OUTPUT);
    pinMode(Pin::INT960_3, OUTPUT);
    pinMode(Pin::BE0, INPUT);
    pinMode(Pin::BE1, INPUT);
    pinMode(Pin::ADS, INPUT);
    pinMode(Pin::BLAST, INPUT);
    pinMode(Pin::HLDA, INPUT);
    pinMode(Pin::READY_SYNC_IN, INPUT);
    pinMode(Pin::HOLD, OUTPUT);
    pinMode(Pin::LOCK, INPUT);
    pinMode(Pin::FAIL, INPUT);
    pinMode(Pin::READY, OUTPUT);
    pinMode(Pin::WR, INPUT);
    // deactivate interrupts
    digitalWrite<Pin::INT960_0, HIGH>();
    digitalWrite<Pin::INT960_1, LOW>();
    digitalWrite<Pin::INT960_2, LOW>();
    digitalWrite<Pin::INT960_3, HIGH>();
    digitalWrite<Pin::HOLD, LOW>();
    digitalWrite<Pin::READY, HIGH>();

    getDirectionRegister<Port::AddressLowest>() = 0;
    getDirectionRegister<Port::AddressLower>() = 0;
    getDirectionRegister<Port::AddressHigher>() = 0;
    getDirectionRegister<Port::AddressHighest>() = 0;
}
void
setupRandomSource() noexcept {
    uint32_t newSeed = 0;
#define X(pin) newSeed += static_cast<uint32_t>(analogRead(pin))
    X(A0);
    X(A1);
    X(A2);
    X(A3);
    X(A4);
    X(A5);
    X(A6);
    X(A6);
    X(A7);
    X(A8);
    X(A9);
    X(A10);
    X(A11);
    X(A12);
    X(A13);
    X(A14);
    X(A15);
#undef X
    randomSeed(newSeed);
}
void
setup() {
    GPIOR0 = 0;
    GPIOR1 = 0;
    GPIOR2 = 0;
    setupRandomSource();
    configurePins();
    configureDataLinesForRead();
    configureInterruptSources();
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(Deception::TWI_ClockRate);
    onboardCache.begin();
    PCLink2.waitForBackingStoreIdle();
    delay(1000);
    digitalWrite<Pin::RESET, HIGH>();
}
[[gnu::always_inline]]
inline void 
waitForNewTransaction() noexcept {
    // clear the READY signal interrupt ahead of waiting for the last
    clearREADYInterrupt();
    do { } while (bit_is_clear(EIFR, ADSFLAG));
    clearADSInterrupt();
}
void 
loop() {
    waitForNewTransaction();
    {
        if constexpr (EnableStateDebuggingPins) {
            digitalWrite<Pin::TransactionStatus, LOW>();
        }
        {
            if (auto address = getAddress(); isReadOperation()) {
                configureDataLinesForRead();
                if (address.isIOOperation()) {
                    doIOTransaction<true>(address);
                } else {
                    doMemoryTransaction<true>(address);
                }
            } else {
                configureDataLinesForWrite();
                if (address.isIOOperation()) {
                    doIOTransaction<false>(address);
                } else {
                    doMemoryTransaction<false>(address);
                }

            }
        }
        if constexpr (EnableStateDebuggingPins) {
            digitalWrite<Pin::TransactionStatus, HIGH>();
        }
    }
}


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
#include <SPI.h>
#include <Deception.h>
#include <SD.h>
#include <EEPROM.h>
#include <RTClib.h>

#include "Types.h"
#include "Pinout.h"
#include "Setup.h"
#include "Pins.h"
#include "OptionalDevice.h"

// stable configuration
constexpr bool UseDirectPortsForDataLines = true;
constexpr bool UseBusKeeper = true;
constexpr auto SerialBaudRate = 115200;
[[gnu::noinline]] void installInitialBootImage() noexcept;
void configureExternalBus() noexcept;

// onboard devices
// EEPROM - [implemented]
// SRAM Cache - [implemented]
// Serial basic interface - [implemented]
// SD Card interface - Partially implemented, need to expose a micro filesystem
// Timer 1, 2, 3, 4, 5 - to do
// USART 2 and 3
//   USART as SPI is another effective mode as well!
// use a ds3231 chip
OptionalDevice<RTC_DS3231> rtc;


template<uint32_t LS>
class PSRAMBackingStore {
    public:
        using SPIDevice = decltype(SPI);
        PSRAMBackingStore(SPIDevice& link) : _link(link) { }
        static constexpr uint32_t LinkSpeed = LS;
        void begin() noexcept;
        size_t read(Address addr, uint8_t* storage, size_t count) noexcept;
        size_t write(Address addr, uint8_t* storage, size_t count) noexcept;
        void waitForBackingStoreIdle() noexcept { }
    private:
        void setAddress(Address address) noexcept;
    private:
        SPIDevice& _link;
};

template<typename T, typename V>
using TypeView = V[sizeof(T) / sizeof(V)];
template<typename T>
using BytesView = TypeView<T, uint8_t>;
template<typename T>
using ShortsView = TypeView<T, uint16_t>;


union SplitWord32 {
    BytesView<uint32_t> bytes;
    ShortsView<uint32_t> halves;
    __uint24 lo24;
    uint32_t full;
    constexpr SplitWord32(uint32_t value = 0) : full(value) { }
    constexpr SplitWord32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : bytes{a, b, c, d} { }
    constexpr bool isIOOperation() const noexcept { return bytes[3] == 0xFE; }
};
static_assert(sizeof(SplitWord32) == sizeof(uint32_t));

union SplitWord16 {
    BytesView<uint16_t> bytes;
    uint16_t full;
    constexpr SplitWord16(uint16_t value = 0) : full(value) { }
    constexpr SplitWord16(uint8_t a, uint8_t b) : bytes{a, b} { }
};

static_assert(sizeof(SplitWord16) == sizeof(uint16_t));


using PrimaryBackingStore = PSRAMBackingStore<5'000'000>;
PrimaryBackingStore psramMemory(SPI);
#define CommunicationPrimitive psramMemory
// this is a special case cache line implementation so we can be very specific
struct FixedCacheLine {
    using Address_t = uint32_t;
    static constexpr auto NumBytes = 16;
    static constexpr auto ShiftAmount = 4;
    static constexpr auto AddressMask = 0xFFFF'FFF0;
private:
    void load(SplitWord32 newAddress) volatile noexcept {
        _dirty = false;
        _valid = true;
        _key.full = newAddress.full;
        (void)CommunicationPrimitive.read(_key.full, const_cast<uint8_t*>(_bytes), NumBytes);
    }
public:
    void clear() volatile noexcept {
        _dirty = false;
        _valid = false;
        _key.full = 0;
        for (auto i = 0u; i < NumBytes; ++i) {
            _bytes[i] = 0;
        }
    }
    volatile uint8_t* sync(SplitWord32 newAddress) volatile noexcept {
        uint8_t offset = newAddress.bytes[0] & 0x0F;
        newAddress.bytes[0] &= 0xF0;
        if (_valid) {
            if (_key.halves[1] != newAddress.halves[1]) {
                if (_dirty) {
                    // just do a compare of the two parts valid and dirty if we
                    // hit 3 or greater then it means we have to perform the
                    // replacement
                    (void)CommunicationPrimitive.write(_key.full, const_cast<uint8_t*>(_bytes), NumBytes);
                }
                load(newAddress);
            }
        } else {
            load(newAddress);
        }
        return &_bytes[offset];
    }
    void markDirty() volatile noexcept { _dirty = true; }
    private:
        uint8_t _bytes[NumBytes];
        SplitWord32 _key;
        bool _valid;
        bool _dirty;
};
[[gnu::address(0xFF00)]] volatile FixedCacheLine externalCacheLine;
union [[gnu::packed]] CH351 {
    struct {
        uint32_t data;
        uint32_t direction;
    } view32;
    struct {
        uint16_t data[2];
        uint16_t direction[2];
    } view16;
    struct {
        uint8_t data[4];
        uint8_t direction[4];
    } view8;
};

static_assert(sizeof(CH351) == 8);
[[gnu::address(0xFF40)]] volatile struct [[gnu::packed]] {
    CH351 addressLines;
    CH351 dataLines;
} interface960;






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
    // Configure falling edge of READY (rising edge can also be quite safe as
    // well
    bitClear(EICRB, READY_ISC0);
    //bitSet(EICRB, READY_ISC0);
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
    toggle<Pins::READY>();
    if constexpr (wait) {
        waitForReady();
    }
}

[[gnu::always_inline]]
[[nodiscard]]
inline bool 
lowerByteEnabled() noexcept {
    return digitalRead<Pins::BE0>() == LOW;
}

[[gnu::always_inline]]
[[nodiscard]]
inline bool 
upperByteEnabled() noexcept {
    return digitalRead<Pins::BE1>() == LOW;
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint8_t 
lowerData() noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        return getInputRegister<Ports::DataLower>();
    } else {
        return interface960.dataLines.view8.data[0];
    }
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint8_t 
upperData() noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        return getInputRegister<Ports::DataUpper>();
    } else {
        return interface960.dataLines.view8.data[1];
    }
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint16_t 
data() noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        union {
            uint8_t bytes[sizeof(uint16_t)];
            uint16_t full;
        } storage;

        storage.bytes[0] = lowerData();
        storage.bytes[1] = upperData();
        return storage.full;
    } else {
        return interface960.dataLines.view16.data[0];
    }
}
[[gnu::always_inline]]
inline void
setUpperData(uint8_t value) noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        getOutputRegister<Ports::DataUpper>() = value;
    } else {
        interface960.dataLines.view8.data[1] = value;
    }
}
[[gnu::always_inline]]
inline void
setLowerData(uint8_t value) noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        getOutputRegister<Ports::DataLower>() = value;
    } else {
        interface960.dataLines.view8.data[0] = value;
    }
}
[[gnu::always_inline]]
inline void
setDataValue(uint16_t value) noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        setLowerData(value);
        setUpperData(value >> 8);
    } else {
        interface960.dataLines.view16.data[0] = value;
    }
}

[[gnu::always_inline]]
inline void
setDataValue(uint8_t lower, uint8_t upper) noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        setLowerData(lower);
        setUpperData(upper);
    } else {
        interface960.dataLines.view8.data[0] = lower;
        interface960.dataLines.view8.data[1] = upper;
    }
}

template<uint16_t value>
[[gnu::always_inline]]
inline void
setDataDirection() noexcept {
    if constexpr (UseDirectPortsForDataLines) {
        getDirectionRegister<Ports::DataLower>() = static_cast<uint8_t>(value);
        getDirectionRegister<Ports::DataUpper>() = static_cast<uint8_t>(value >> 8);
    } else {
        interface960.dataLines.view16.direction[0] = value;
    }
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
    return digitalRead<Pins::WR>() == LOW;
}
[[gnu::always_inline]] inline bool isLastWordOfTransaction() noexcept {
    return digitalRead<Pins::BLAST>() == LOW;
}
template<bool isReadOperation, bool signalReadyAtEnd = true>
inline void
doNothingOperation() noexcept {
    if constexpr(isReadOperation) {
        setDataValue(0);
    }
    while (!isLastWordOfTransaction()) {
        signalReady();
    }
    if constexpr (signalReadyAtEnd) {
        signalReady();
    }
}
inline void
send32BitConstant(uint32_t value) noexcept {
    do {
        setDataValue(value);
        if (isLastWordOfTransaction()) {
            break;
        }
        signalReady<false>();
        auto upper = static_cast<uint16_t>(value >> 16);
        waitForReady();
        setDataValue(upper);
        if (isLastWordOfTransaction()) {
            break;
        }
        signalReady();
        doNothingOperation<true, false>();
    } while (false);
    signalReady();
}

inline void
send16BitValue(uint16_t value) noexcept {
    do {
        setDataValue(value);
        if (isLastWordOfTransaction()) {
            break;
        }
        signalReady();
        doNothingOperation<true, false>();
    } while (false);
    signalReady();
}
inline void
send16BitValue(uint8_t lo, uint8_t hi) noexcept {
    do {
        setLowerData(lo);
        setUpperData(hi);
        if (isLastWordOfTransaction()) {
            break;
        }
        signalReady();
        doNothingOperation<true, false>();
    } while (false);
    signalReady();
}
template<bool readOperation>
inline void transmitValue(uint32_t value, TreatAs<uint32_t>) noexcept {
    if constexpr (readOperation) {
        send32BitConstant(value);
    } else {
        doNothingOperation<readOperation>();
    }
}

template<bool readOperation>
inline void transmitValue(uint16_t value, TreatAs<uint16_t>) noexcept {
    if constexpr (readOperation) {
        send16BitValue(value);
    } else {
        doNothingOperation<readOperation>();
    }
}

template<bool readOperation>
inline void transmitValue(int16_t value, TreatAs<int16_t>) noexcept {
    union {
        int16_t value;
        uint16_t to;
    } conversion;
    conversion.value = value;
    transmitValue<readOperation>(conversion.to, TreatAs<uint16_t>{});
}

template<bool readOperation>
inline void transmitValue(bool value, TreatAs<bool>) noexcept {
    if constexpr (readOperation) {
        send32BitConstant(value ? 0xFFFF'FFFF : 0);
    } else {
        doNothingOperation<readOperation>();
    }
}
template<bool readOperation>
inline void transmitValue(float value, TreatAs<float>) noexcept {
    union {
        float flt;
        uint32_t raw;
    } converter;
    converter.flt = value;
    transmitValue<readOperation>(converter.raw, TreatAs<uint32_t>{});
}
template<bool readOperation>
inline void transmitValue(uint64_t value, TreatAs<uint64_t>) noexcept {
    if constexpr (readOperation) {
        do {
            setDataValue(static_cast<uint16_t>(value));
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            auto next = static_cast<uint16_t>(value >> 16);
            waitForReady();
            setDataValue(next);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            next = static_cast<uint16_t>(value >> 32);
            waitForReady();
            setDataValue(next);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            next = static_cast<uint16_t>(value >> 48);
            waitForReady();
            setDataValue(next);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            doNothingOperation<readOperation, false>();
        } while (false);
        signalReady();
    } else {
        doNothingOperation<readOperation>();
    }
}
template<bool readOperation>
inline void transmitValue(float a, float b, TreatAs<uint64_t>) noexcept {
    union {
        float components[2];
        uint64_t whole;
    } container;
    container.components[0] = a;
    container.components[1] = b;
    transmitValue<readOperation>(container.whole, TreatAs<uint64_t>{});
}
template<bool readOperation, typename T>
inline void handleAvailableRequest(const T& item) noexcept {
    transmitValue<readOperation>(item.valid(), TreatAs<bool>{});
}
template<bool readOperation>
inline void
doRTCOperation(uint8_t offset) noexcept {
    // you cannot stream data from the rtc device using quad, triple, and long load/store
    switch (offset) {
        case 0x00: // available
            handleAvailableRequest<readOperation>(rtc);
            break;
        case 0x04: // unixtime
            transmitValue<readOperation>(rtc->now().unixtime(), TreatAs<uint32_t>{});
            break;
        case 0x08: // secondstime
            transmitValue<readOperation>(rtc->now().secondstime(), TreatAs<uint32_t>{});
            break;
        case 0x0C: // temperature
            transmitValue<readOperation>(rtc->getTemperature(), TreatAs<float>{});
            break;
        case 0x10: 
            if constexpr (readOperation) {
                // now
                auto now = rtc->now();
                setDataValue(now.second(), now.minute());
                if (isLastWordOfTransaction()) {
                    signalReady();
                    break;
                }
                signalReady();
                setDataValue(now.hour(), now.day());
                if (isLastWordOfTransaction()) {
                    signalReady();
                    break;
                }
                signalReady();
                setDataValue(now.month(), 0);
                if (isLastWordOfTransaction()) {
                    signalReady();
                    break;
                }
                signalReady();
                setDataValue(now.year());
                if (isLastWordOfTransaction()) {
                    signalReady();
                    break;
                }
                signalReady();
            }
            doNothingOperation<readOperation>();
            break;
        case 0x20: // 32k configure, read returns enabled, write configures
            if constexpr (readOperation) {
                transmitValue<readOperation>(rtc->isEnabled32K(), TreatAs<bool>{});
            } else {
                if (lowerData() != 0) {
                    rtc->enable32K();
                } else {
                    rtc->disable32K();
                }
                doNothingOperation<readOperation>();
            }
            break;
        default:
            doNothingOperation<readOperation>();
            break;
    }
}
template<bool readOperation>
inline void
handleSerialDeviceInterface(uint8_t offset, HardwareSerial& device) noexcept {
    // this is a wrapper interface over a given hardware serial device
    // It should provide extra functions for making execution as easy as
    // possible
    switch (offset) {
        case 0x00: // available
            transmitValue<readOperation>(true, TreatAs<bool>{});
            break;
        default:
            doNothingOperation<readOperation>();
            break;
    }
}

template<bool readOperation>
inline void
handleEEPROMDevice(uint16_t index) noexcept {
    do {
        index &= 0xFFF;
        if constexpr (readOperation) {
            uint16_t curr = 0;
            setDataValue(EEPROM.get<uint16_t>(index, curr));
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+2, curr);
            waitForReady();
            setDataValue(curr);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+4, curr);
            waitForReady();
            setDataValue(curr);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+6, curr);
            waitForReady();
            setDataValue(curr);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+8, curr);
            waitForReady();
            setDataValue(curr);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+10, curr);
            waitForReady();
            setDataValue(curr);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+12, curr);
            waitForReady();
            setDataValue(curr);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            (void)EEPROM.get<uint16_t>(index+14, curr);
            waitForReady();
            setDataValue(curr);
        } else {
            /// @todo optimize
            if (lowerByteEnabled()) {
                EEPROM.update(index, lowerData());
            }
            if (upperByteEnabled()) {
                EEPROM.update(index+1, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+2, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+3, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+4, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+5, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+6, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+7, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+8, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+9, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+10, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+11, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+12, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+13, upperData());
            }
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady();
            EEPROM.update(index+14, lowerData());
            if (upperByteEnabled()) {
                EEPROM.update(index+15, upperData());
            }
        }
    } while (false);
    signalReady();
}
constexpr auto SRAMCacheCapacity = 2048;
uint8_t sramCache[SRAMCacheCapacity] = { 0 };
template<bool readOperation>
inline void
handleSRAMDevice(uint16_t address) noexcept {
    uint8_t lo, hi;
    do {
        address &= 0x7FF;
        if constexpr (readOperation) {
            lo = sramCache[address + 0];
            hi = sramCache[address + 1];
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) { 
                break; 
            } 
            signalReady<false>();
            lo = sramCache[address + 2];
            hi = sramCache[address + 3];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) { 
                break; 
            } 
            signalReady<false>();
            lo = sramCache[address + 4];
            hi = sramCache[address + 5];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) { 
                break; 
            } 
            signalReady<false>();
            lo = sramCache[address + 6];
            hi = sramCache[address + 7];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) { 
                break; 
            } 
            signalReady<false>();
            lo = sramCache[address + 8];
            hi = sramCache[address + 9];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) { 
                break; 
            } 
            signalReady<false>();
            lo = sramCache[address + 10];
            hi = sramCache[address + 11];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) { 
                break; 
            } 
            signalReady<false>();
            lo = sramCache[address + 12];
            hi = sramCache[address + 13];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
        } else {
#define X(a) \
            if (lowerByteEnabled()) { \
                sramCache[address + a + 0] = lowerData(); \
            } \
            if (upperByteEnabled()) { \
                sramCache[address + a + 1] = upperData(); \
            } \
            if (isLastWordOfTransaction()) { \
                break; \
            } \
            signalReady()
            X(0);
            X(2);
            X(4);
            X(6);
            X(8);
            X(10);
            X(12);
#undef X
            lo = lowerData();
            hi = upperData();
            if (lowerByteEnabled()) {
                sramCache[address + 0] = lo;
            }
            if (isLastWordOfTransaction) {
                if (upperByteEnabled()) {
                    sramCache[address + 1] = hi;
                }
                break;
            }
            signalReady<false>();
            sramCache[address + 1] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    sramCache[address + 3] = hi;
                }
                sramCache[address + 2] = lo;
                break;
            }
            signalReady<false>();
            sramCache[address + 2] = lo;
            sramCache[address + 3] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    sramCache[address + 4] = hi;
                }
                sramCache[address + 5] = lo;
                break;
            }
            signalReady<false>();
            sramCache[address + 4] = lo;
            sramCache[address + 5] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    sramCache[address + 7] = hi;
                }
                sramCache[address + 6] = lo;
                break;
            }
            signalReady<false>();
            sramCache[address + 6] = lo;
            sramCache[address + 7] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    sramCache[address + 9] = hi;
                }
                sramCache[address + 8] = lo;
                break;
            }
            signalReady<false>();
            sramCache[address + 8] = lo;
            sramCache[address + 9] = hi;
            waitForReady();

            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    sramCache[address + 11] = hi;
                }
                sramCache[address + 10] = lo;
                break;
            }
            signalReady<false>();
            sramCache[address + 10] = lo;
            sramCache[address + 11] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    sramCache[address + 13] = hi;
                }
                sramCache[address + 12] = lo;
                break;
            }
            signalReady<false>();
            sramCache[address + 12] = lo;
            sramCache[address + 13] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            if (upperByteEnabled()) {
                sramCache[address + 15] = hi;
            }
            sramCache[address + 14] = lo;
        }
    } while (false);
    signalReady();
}
struct [[gnu::packed]] DeviceInformation {
    union {
        uint8_t bytes[16];
        struct {
            uint8_t kind;
        };
    };
    template<bool readOperation>
    void transmit(uint8_t offset = 0) noexcept {
        if constexpr (readOperation) {
            switch (offset & 0b0000'1110) {
                case 0x0:
                    setLowerData(bytes[0]);
                    setUpperData(bytes[1]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0x2:
                    setLowerData(bytes[2]);
                    setUpperData(bytes[3]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0x4:
                    setLowerData(bytes[4]);
                    setUpperData(bytes[5]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0x6:
                    setLowerData(bytes[6]);
                    setUpperData(bytes[7]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0x8:
                    setLowerData(bytes[8]);
                    setUpperData(bytes[9]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0xa:
                    setLowerData(bytes[10]);
                    setUpperData(bytes[11]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0xc:
                    setLowerData(bytes[12]);
                    setUpperData(bytes[13]);
                    if (isLastWordOfTransaction()) {
                        break;
                    }
                    signalReady();
                case 0xe:
                    setLowerData(bytes[14]);
                    setUpperData(bytes[15]);
                    break;
            }
            signalReady();
        } else {
            doNothingOperation<readOperation>();
        }
    }
};
static_assert(sizeof(DeviceInformation) == 16);
template<bool readOperation>
inline void 
handleBuiltinDevices(uint8_t offset) noexcept {
    switch (offset) {
        case 0x00:
            transmitValue<readOperation>(F_CPU, TreatAs<uint32_t>{});
            break;
        case 0x04:
            transmitValue<readOperation>(F_CPU / 2, TreatAs<uint32_t>{});
            break;
        case 0x08:
            if constexpr (readOperation) {
                send16BitValue(Serial.read());
            } else {
                Serial.write(lowerData());
                doNothingOperation<readOperation>();
            }
            break;
        case 0x0c:
            if constexpr (!readOperation) {
                Serial.flush();
            }
            doNothingOperation<readOperation>();
            break;
        case 0x10:
            transmitValue<readOperation>(millis(), TreatAs<uint32_t>{});
            break;
        case 0x14:
            transmitValue<readOperation>(micros(), TreatAs<uint32_t>{});
            break;
        case 0x18: // EEPROM Base Address
            transmitValue<readOperation>(0xFE001000, TreatAs<uint32_t>{});
            break;
        case 0x1c: // SRAM Base Address
            transmitValue<readOperation>(0xFE000800, TreatAs<uint32_t>{});
            break;
        case 0x20: // RTC Base Address
            transmitValue<readOperation>(0xFE000200, TreatAs<uint32_t>{});
            break;
        case 0x24: // EEPROM Capacity
            transmitValue<readOperation>(EEPROM.length(), TreatAs<uint16_t>{});
            break;
        case 0x26: // SRAM Capacity
            transmitValue<readOperation>(SRAMCacheCapacity, TreatAs<uint16_t>{});
            break;
        default:
            doNothingOperation<readOperation>();
            break;
    }
}
template<bool readOperation>
inline void
doIOTransaction(SplitWord32 address) noexcept {
    // only dispatch with bytes[1] 
    switch (address.bytes[1]) {
        case 0x00:
            handleBuiltinDevices<readOperation>(address.bytes[0]);
            break;
        case 0x01:
            handleSerialDeviceInterface<readOperation>(address.bytes[0], Serial);
            break;
        case 0x02:
            doRTCOperation<readOperation>(address.bytes[0]);
            break;
        case 0x08 ... 0x0F: // 2k sram cache
            handleSRAMDevice<readOperation>(address.halves[0]);
            break;
        case 0x10 ... 0x1F:
            handleEEPROMDevice<readOperation>(address.halves[0]);
            break;
        default:
            doNothingOperation<readOperation>();
            break;
    }
}
template<bool readOperation>
inline void
doMemoryTransaction(SplitWord32 address) noexcept {
    using MemoryPointer = volatile uint8_t*;
    MemoryPointer ptr = externalCacheLine.sync(address.full);
    do {
        if constexpr (readOperation) {
            auto lo = ptr[0];
            auto hi = ptr[1];
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[2];
            hi = ptr[3];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[4];
            hi = ptr[5];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[6];
            hi = ptr[7];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[8];
            hi = ptr[9];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[10];
            hi = ptr[11];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[12];
            hi = ptr[13];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
            if (isLastWordOfTransaction()) {
                break;
            }
            signalReady<false>();
            lo = ptr[14];
            hi = ptr[15];
            waitForReady();
            setLowerData(lo);
            setUpperData(hi);
        } else {
            externalCacheLine.markDirty();
            auto lo = lowerData();
            auto hi = upperData();
            if (lowerByteEnabled()) {
                ptr[0] = lo;
            }
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[1] = hi;
                }
                signalReady();
                return;
            }
            signalReady<false>();
            ptr[1] = hi;
            waitForReady();
            // we can safely ignore checking BE0 since we flowed into this
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[3] = hi;
                }
                ptr[2] = lo;
                break;
            }
            signalReady<false>();
            ptr[2] = lo;
            ptr[3] = hi;
            waitForReady();
            // we can safely ignore checking BE0 since we flowed into this
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[5] = hi;
                }
                ptr[4] = lo;
                break;
            }
            signalReady<false>();
            ptr[4] = lo;
            ptr[5] = hi;
            waitForReady();
            // we can safely ignore checking BE0 since we flowed into this
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[7] = hi;
                }
                ptr[6] = lo;
                break;
            }
            signalReady<false>();
            ptr[6] = lo;
            ptr[7] = hi;
            waitForReady();
            // we can safely ignore checking BE0 since we flowed into this
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[9] = hi;
                }
                ptr[8] = lo;
                break;
            }
            signalReady<false>();
            ptr[8] = lo;
            ptr[9] = hi;
            waitForReady();
            // we can safely ignore checking BE0 since we flowed into this
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[11] = hi;
                }
                ptr[10] = lo;
                break;
            }
            signalReady<false>();
            ptr[10] = lo;
            ptr[11] = hi;
            waitForReady();
            // we can safely ignore checking BE0 since we flowed into this
            lo = lowerData();
            hi = upperData();
            if (isLastWordOfTransaction()) {
                if (upperByteEnabled()) {
                    ptr[13] = hi;
                }
                ptr[12] = lo;
                break;
            }
            signalReady<false>();
            ptr[12] = lo;
            ptr[13] = hi;
            waitForReady();
            lo = lowerData();
            hi = upperData();
            // we can safely ignore checking BE0 since we flowed into this
            if (upperByteEnabled()) {
                ptr[15] = hi;
            }
            ptr[14] = lo;
        }
    } while (false);
    signalReady();

}

inline
SplitWord32
getAddress() noexcept {
    return { interface960.addressLines.view32.data };
}

void
configurePins() noexcept {
    pinMode(Pins::RESET, OUTPUT);
    digitalWrite<Pins::RESET, LOW>();
    pinMode(Pins::PSRAM_EN, OUTPUT);
    digitalWrite<Pins::PSRAM_EN, HIGH>();
    getDirectionRegister<Ports::PSRAMSel>() = 0xFF;
    getOutputRegister<Ports::PSRAMSel>() = 0x00;
    pinMode(Pins::INT960_0, OUTPUT);
    pinMode(Pins::INT960_1, OUTPUT);
    pinMode(Pins::INT960_2, OUTPUT);
    pinMode(Pins::INT960_3, OUTPUT);
    pinMode(Pins::BE0, INPUT);
    pinMode(Pins::BE1, INPUT);
    pinMode(Pins::ADS, INPUT);
    pinMode(Pins::BLAST, INPUT);
    pinMode(Pins::HLDA, INPUT);
    pinMode(Pins::READY_SYNC_IN, INPUT);
    pinMode(Pins::HOLD, OUTPUT);
    pinMode(Pins::READY, OUTPUT);
    pinMode(Pins::WR, INPUT);
    // deactivate interrupts
    digitalWrite<Pins::INT960_0, HIGH>();
    digitalWrite<Pins::INT960_1, LOW>();
    digitalWrite<Pins::INT960_2, LOW>();
    digitalWrite<Pins::INT960_3, HIGH>();
    digitalWrite<Pins::HOLD, LOW>();
    digitalWrite<Pins::READY, HIGH>();

    // configure the address lines as output to start
    interface960.addressLines.view32.direction = 0xFFFF'FFFF;
    // then setup the external bus, it is necessary for the next step
}
void sanityCheckHardwareAcceleratedCacheLine() noexcept;
void setupExternalDevices() noexcept;
void configureRandomSeed() noexcept;
void
setup() {
    configureRandomSeed();
    configurePins();
    configureExternalBus();
    configureDataLinesForRead();
    if constexpr (!UseDirectPortsForDataLines) {
        // make sure that we set data lower and data upper direct ports to be inputs to be safe
        getDirectionRegister<Ports::DataLower>() = 0;
        getDirectionRegister<Ports::DataUpper>() = 0;
    }
    configureInterruptSources();
    Serial.begin(SerialBaudRate);
    Serial.print(F("Serial Up @ "));
    Serial.println(SerialBaudRate);
    EEPROM.begin();
    SPI.begin();
    Wire.begin();
    Wire.setClock(Deception::TWI_ClockRate);
    psramMemory.begin();
    installInitialBootImage();
    setupExternalDevices();
    CommunicationPrimitive.waitForBackingStoreIdle();
    sanityCheckHardwareAcceleratedCacheLine();
    // do not cache anything to start with as we should instead just be ready
    // to get data from psram instead
    interface960.addressLines.view32.direction = 0;
    digitalWrite<Pins::RESET, HIGH>();
}
void
setupRTC() noexcept {
    Serial.println(F("Setting up RTC"));
    // setup the RTC
    if (!rtc.begin()) {
        Serial.println(F("Couldn't find RTC"));
        Serial.flush();
    } else {
        if (rtc->lostPower()) {
            Serial.println(F("RTC lost power, setting time"));
            rtc->adjust(DateTime(F(__DATE__), F(__TIME__)));
        }

        DateTime now = rtc->now();
        Serial.printf(F(" since midnight 1/1/1970 = %lds = %ldd\n"), now.unixtime(), now.unixtime() / 86400L);
    }
}
void
setupExternalDevices() noexcept {
    setupRTC();
}
void 
loop() {
    do {
        // clear the READY signal interrupt ahead of waiting for the last
        clearREADYInterrupt();
        do { } while (bit_is_clear(EIFR, ADSFLAG));
        clearADSInterrupt();
        if (auto address = getAddress(); isReadOperation()) {
            configureDataLinesForRead();
            switch (address.bytes[3]) {
                case 0x00 ... 0x7f:
                    doMemoryTransaction<true>(address);
                    break;
                case 0xFE:
                    doIOTransaction<true>(address);
                    break;
                default:
                    configureDataLinesForWrite();
                    doNothingOperation<false>();
                    break;
            }
        } else {
            configureDataLinesForWrite();
            switch (address.bytes[3]) {
                case 0x00 ... 0x7f:
                    doMemoryTransaction<false>(address);
                    break;
                case 0xFE:
                    doIOTransaction<false>(address);
                    break;
                default:
                    doNothingOperation<false>();
                    break;
            }
        }
    } while(true);
}


template<uint32_t LS>
void
PSRAMBackingStore<LS>::begin() noexcept {
    auto activatePSRAM = [this](bool pullID, bool performSanityChecking) {
        digitalWrite<Pins::PSRAM_EN, LOW>();
        _link.transfer(0x99);
        digitalWrite<Pins::PSRAM_EN, HIGH>();
        digitalWrite<Pins::PSRAM_EN, LOW>();
        _link.transfer(0x66);
        digitalWrite<Pins::PSRAM_EN, HIGH>();
        delay(100);
        if (pullID) {
            union Id {
                uint64_t full;
                struct {
                    uint32_t eidLo;
                    uint16_t eidHi;
                    uint8_t kgd;
                    uint8_t mfid;
                };
                uint8_t bytes[8];
            };
            Id tmp;
            tmp.full = 0;
            digitalWrite<Pins::PSRAM_EN, LOW>();
            _link.transfer(0x9f);
            _link.transfer(0);
            _link.transfer(0);
            _link.transfer(0);
            _link.transfer(tmp.bytes, 8);
            digitalWrite<Pins::PSRAM_EN, HIGH>();
            delay(100);
            Serial.printf(F("MFID: 0x%x, KGD: 0x%x, EIDHi: 0x%x, EIDLo: 0x%lx\n"), tmp.mfid, tmp.kgd, tmp.eidHi, tmp.eidLo);
        }
        if (performSanityChecking) {
            // now we need to do some amount of sanity checking to see if we can do
            // a read and write operation
            uint8_t storage[16] = { 0 };
            uint8_t storage2[16] = { 0 };
            for (int i = 0; i < 16; ++i) {
                storage[i] = random();
                storage2[i] = storage[i];
            }
            (void)write(0, storage, 16);
            for (int i = 0; i < 16; ++i) {
                storage[i] = 0;
            }
            (void)read(0, storage, 16);
            for (int i = 0; i < 16; ++i) {
                auto against = storage2[i];
                auto compare = storage[i];
                if (against != compare) {
                    Serial.printf(F("@0x%x: (in)0x%x != (control)0x%x\n"), i, compare, against);
                } 
            }
        }

    };
    delay(1000); // make sure that the waiting duration is enough for powerup
    _link.beginTransaction(SPISettings{LS, MSBFIRST, SPI_MODE0});
    for (uint8_t i = 0; i < 8; ++i) {
        // can't use setAddress since that is setup for direct addresses
        getOutputRegister<Ports::PSRAMSel>() = i;
        activatePSRAM(true, true);
    }
    _link.endTransaction();
    getOutputRegister<Ports::PSRAMSel>() = 0;

}
void
installInitialBootImage() noexcept {
    if (!SD.begin(static_cast<int>(Pins::SD_EN))) {
        Serial.println(F("No SDCard found!"));
    } else {
        Serial.println(F("Found an SDCard, will try to transfer the contents of prog.bin to onboard psram"));
        auto f = SD.open(F("prog.bin"), FILE_READ); 
        if (!f) {
            Serial.println(F("Could not open prog.bin...skipping!"));
        } else {
            Serial.println(F("Found prog.bin..."));
            if (f.size() <= 0x0100'0000) {
                Serial.println(F("Transferring prog.bin to memory"));
                static constexpr auto ByteCount = 32;
                uint8_t dataBytes[ByteCount] = { 0 };
                for (uint32_t i = 0, j = 0; i < f.size(); i+=ByteCount, ++j) {
                    auto count = f.read(dataBytes, ByteCount);
                    // make a temporary copy for compare purposes
                    psramMemory.write(i, dataBytes, count);
                    // put a blip out every 64k
                    if ((j & 0xFF) == 0) {
                        Serial.print('.');
                    }
                }
                Serial.println(F("Transfer complete!"));
                Serial.println(F("Header Contents:"));
                psramMemory.read(0, dataBytes, ByteCount);
                auto* header = reinterpret_cast<uint32_t*>(dataBytes);
                for (size_t i = 0; i < (ByteCount / sizeof(uint32_t)); ++i) {
                    Serial.printf(F("\t0x%x: 0x%x\n"), i, header[i]);
                }
            } else {
                Serial.println(F("prog.bin is too large to fit in 16 megabytes!"));
            }
            f.close();
        }
    }
}

template<uint32_t LS>
void 
PSRAMBackingStore<LS>::setAddress(Address address) noexcept {
#if 0
    uint8_t addr = static_cast<uint8_t>(address >> 23) & 0b111;
#else
    uint8_t addr = static_cast<uint8_t>(address >> 24);
    addr <<= 1;
    if (static_cast<uint8_t>(address >> 16) & 0b1000'0000) {
        addr |= 0b1;
    } else {
        addr &= 0b1111'1110;
    }
#endif
    getOutputRegister<Ports::PSRAMSel>() = addr;
}

template<uint32_t LS>
inline size_t
PSRAMBackingStore<LS>::read(Address addr, uint8_t* storage, size_t count) noexcept {
    setAddress(addr);
    _link.beginTransaction(SPISettings{LS, MSBFIRST, SPI_MODE0});
    digitalWrite<Pins::PSRAM_EN, LOW>();
    _link.transfer(0x03);
    _link.transfer(static_cast<uint8_t>(addr >> 16));
    _link.transfer(static_cast<uint8_t>(addr >> 8));
    _link.transfer(static_cast<uint8_t>(addr));
    _link.transfer(storage, count);
    digitalWrite<Pins::PSRAM_EN, HIGH>();
    _link.endTransaction();
    return count;
}

template<uint32_t LS>
inline size_t
PSRAMBackingStore<LS>::write(Address addr, uint8_t* storage, size_t count) noexcept {
    setAddress(addr);
    _link.beginTransaction(SPISettings{LS, MSBFIRST, SPI_MODE0});
    digitalWrite<Pins::PSRAM_EN, LOW>();
    _link.transfer(0x02);
    _link.transfer(static_cast<uint8_t>(addr >> 16));
    _link.transfer(static_cast<uint8_t>(addr >> 8));
    _link.transfer(static_cast<uint8_t>(addr));
    _link.transfer(storage, count);
    digitalWrite<Pins::PSRAM_EN, HIGH>();
    _link.endTransaction();
    return count;
}

void
sanityCheckHardwareAcceleratedCacheLine() noexcept {
    Serial.println(F("Using External Hardware Accelerated Cache!"));
    Serial.println(F("Zeroing out cache memory"));
    for (uint32_t i = 0; i < (1024ul * 1024ul); i += 16) {
        interface960.addressLines.view32.data = i;
        if (static_cast<uint16_t>(i) == 0) {
            Serial.print('.');
        }
        externalCacheLine.clear();
    }
    Serial.println(F("DONE!"));
}

void 
configureExternalBus() noexcept {
    // one cycle wait to be on the safe side
    bitClear(XMCRA, SRW11);
    bitSet(XMCRA, SRW10);
    bitClear(XMCRA, SRW01);
    bitSet(XMCRA, SRW00);
    // half and half sector limits (doesn't really matter since it will an
    // 8-bit space
    bitClear(XMCRA, SRL0);
    bitClear(XMCRA, SRL1);
    bitSet(XMCRA, SRL2); 
    // no high address bits!
    bitSet(XMCRB, XMM0); 
    bitSet(XMCRB, XMM1); 
    bitSet(XMCRB, XMM2); 
    if constexpr (UseBusKeeper) {
        bitSet(XMCRB, XMBK);
    } else {
        bitClear(XMCRB, XMBK); 
    }

    bitSet(XMCRA, SRE); // enable the EBI
}

void
configureRandomSeed() noexcept {
    uint32_t newSeed = analogRead(A0);
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




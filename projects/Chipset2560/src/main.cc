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
#include <RTClib.h>
#include <SparkFun_Alphanumeric_Display.h>
#include <Adafruit_Si7021.h>

#include "Types.h"
#include "Pinout.h"
#include "Setup.h"
template<typename T>
struct OptionalDevice {
    public:
        template<typename ... Args>
        OptionalDevice(Args&& ... args) : _device(args...) { }
        constexpr bool valid() const noexcept {
            return _valid;
        }
        T& get() noexcept {
            return _device;
        }
        template<typename ... Args>
        bool begin(Args&& ... args) noexcept {
            _valid = _device.begin(args...);
            return _valid;
        }
        T& operator*() const noexcept { return _device; }
        T* operator->() noexcept { return &_device; }
        const T* operator->() const noexcept { return &_device; }
        explicit operator bool() const noexcept { return _valid; }
    private:
        bool _valid = false;
        T _device;
};
// use a ds3231 chip
OptionalDevice<RTC_DS3231> rtc;
OptionalDevice<HT16K33> numericDisplay;
OptionalDevice<Adafruit_Si7021> sensor_si7021;
namespace Pins {
    constexpr auto SD_EN = Pin::PortB0;
    constexpr auto INT960_0 = Pin::PortB4;
    constexpr auto INT960_1 = Pin::PortB5;
    constexpr auto INT960_2 = Pin::PortB6;
    constexpr auto INT960_3 = Pin::PortB7;
    constexpr auto BE0 = Pin::PortD4;
    constexpr auto BE1 = Pin::PortD5;
    constexpr auto BLAST = Pin::PortD6;
    constexpr auto WR = Pin::PortD7;
    constexpr auto RESET = Pin::PortE2;
    constexpr auto HOLD = Pin::PortE3;
    constexpr auto ADS = Pin::PortE4;
    constexpr auto HLDA = Pin::PortE5;
    constexpr auto LOCK = Pin::PortE6;
    constexpr auto READY_SYNC_IN = Pin::PortE7;
    constexpr auto READY = Pin::PortG4;
    // PortG3 uncommitted
    // PortG5 uncommitted
    // PortD2 uncommitted
    // PortD3 uncommitted
}
namespace Ports {
    constexpr auto DataLower = Port::C;
    constexpr auto DataUpper = Port::F;
    constexpr auto AddressLowest = Port::H;
    constexpr auto AddressLower = Port::K;
    constexpr auto AddressHigher = Port::J;
    constexpr auto AddressHighest = Port::L;
}

Deception::TwoWireBackingStore PCLink2(Wire, Deception::TWI_MemoryControllerIndex);
using CacheAddress = __uint24;
//using CacheAddress = uint32_t;
constexpr auto CacheLineCount = 256;
using CacheLine = Deception::CacheLine16<CacheAddress, Deception::TwoWireBackingStore>;
using DataCache = Deception::DirectMappedCache<CacheLineCount, CacheLine>;
DataCache onboardCache;
static_assert(sizeof(CacheLine) <= 32);
//[[gnu::address(0xFF00)]] volatile CacheLine externalCacheLine;
union [[gnu::packed]] SplitWord32 {
    uint8_t bytes[sizeof(uint32_t) / sizeof(uint8_t)];
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    __uint24 lo24;
    uint32_t full;
    constexpr SplitWord32(uint32_t value = 0) : full(value) { }
    constexpr SplitWord32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : bytes{a, b, c, d} { }
    constexpr bool isIOOperation() const noexcept { return bytes[3] == 0xFE; }
    constexpr auto getCacheOffset() const noexcept { return DataCache::computeOffset(bytes[0]); }
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
    return getInputRegister<Ports::DataLower>();
}

[[gnu::always_inline]]
[[nodiscard]]
inline uint8_t 
upperData() noexcept {
    return getInputRegister<Ports::DataUpper>();
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
    getOutputRegister<Ports::DataUpper>() = value;
}
[[gnu::always_inline]]
inline void
setLowerData(uint8_t value) noexcept {
    getOutputRegister<Ports::DataLower>() = value;
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
    getDirectionRegister<Ports::DataLower>() = static_cast<uint8_t>(value);
    getDirectionRegister<Ports::DataUpper>() = static_cast<uint8_t>(value >> 8);
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
    auto& line = onboardCache.find(PCLink2, address.lo24);
    auto* ptr = line.getLineData(address.getCacheOffset());
    if constexpr (readOperation) {
        uint16_t* ptr16 = reinterpret_cast<uint16_t*>(ptr);
        auto val = ptr16[0];
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[1];
        waitForReady();
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[2];
        waitForReady();
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[3];
        waitForReady();
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[4];
        waitForReady();
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[5];
        waitForReady();
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[6];
        waitForReady();
        setDataValue(val);
        if (isLastWordOfTransaction()) {
            goto ReadMemoryDone;
        }
        signalReady<false>();
        val = ptr16[7];
        waitForReady();
        setDataValue(val);
ReadMemoryDone:
        signalReady();
    } else {
        line.markDirty();
        auto lo = lowerData();
        auto hi = upperData();
        if (lowerByteEnabled()) ptr[0] = lo;
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[1] = hi;
            signalReady<false>();
            goto WriteMemoryDone;
        }
        signalReady<false>();
        ptr[1] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        lo = lowerData();
        hi = upperData();
        if (isLastWordOfTransaction()) {
            if (upperByteEnabled()) ptr[3] = hi;
            signalReady<false>();
            ptr[2] = lo;
            goto WriteMemoryDone;
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
            goto WriteMemoryDone;
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
            goto WriteMemoryDone;
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
            goto WriteMemoryDone;
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
            goto WriteMemoryDone;
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
            goto WriteMemoryDone;
        }
        signalReady<false>();
        ptr[12] = lo;
        ptr[13] = hi;
        waitForReady();
        // we can safely ignore checking BE0 since we flowed into this
        ptr[14] = lowerData();
        if (upperByteEnabled()) ptr[15] = upperData();
        signalReady<false>();
WriteMemoryDone:
        waitForReady();
    }
}

inline
SplitWord32
getAddress() noexcept {
    return { 
        getInputRegister<Ports::AddressLowest>(),
        getInputRegister<Ports::AddressLower>(),
        getInputRegister<Ports::AddressHigher>(),
        getInputRegister<Ports::AddressHighest>()
    };
}
void 
configureExternalBus() noexcept {
    // no wait states
    bitClear(XMCRA, SRW11);
    bitClear(XMCRA, SRW10);
    bitClear(XMCRA, SRW01);
    bitClear(XMCRA, SRW00);
    // half and half sector limits (doesn't really matter since it will an
    // 8-bit space
    bitClear(XMCRA, SRL0);
    bitClear(XMCRA, SRL1);
    bitSet(XMCRA, SRL2); 
    // no high address bits!
    bitSet(XMCRB, XMM0); 
    bitSet(XMCRB, XMM1); 
    bitSet(XMCRB, XMM2); 
    bitClear(XMCRB, XMBK); // no bus keeper
    bitSet(XMCRA, SRE); // enable the EBI
}

void
configurePins() noexcept {
    pinMode(Pins::RESET, OUTPUT);
    digitalWrite<Pins::RESET, LOW>();
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

    getDirectionRegister<Ports::AddressLowest>() = 0;
    getDirectionRegister<Ports::AddressLower>() = 0;
    getDirectionRegister<Ports::AddressHigher>() = 0;
    getDirectionRegister<Ports::AddressHighest>() = 0;
    configureExternalBus();
}
const char *daysOfTheWeek[] {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
};
void
setup() {
    GPIOR0 = 0;
    GPIOR1 = 0;
    GPIOR2 = 0;
    configurePins();
    configureDataLinesForRead();
    configureInterruptSources();
    Serial.begin(115200);
    Serial.println(F("SERIAL UP @ 115200"));
    //Serial1.begin(9600);
    SPI.begin();
    Wire.begin();
    //Wire.setClock(Deception::TWI_ClockRate);
    onboardCache.begin();
#if 0
    PCLink2.waitForBackingStoreIdle();
    // okay now we need to setup the cache so that I can eliminate the valid
    // bit. This is done by seeding the cache with teh first 4096 bytes
    for (Address i = 0; i < DataCache::NumCacheBytes; i += DataCache::NumBytesPerLine) {
        Serial.printf(F("Seeding 0x%lx\n"), i);
        onboardCache.seed(PCLink2, i);
    }
#endif
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
        Serial.printf(F("%d/%d/%d (%s) %d:%d:%d\n"), now.year(), now.month(), now.day(), daysOfTheWeek[now.dayOfTheWeek()], now.hour(), now.minute(), now.second());
        Serial.printf(F(" since midnight 1/1/1970 = %lds = %ldd\n"), now.unixtime(), now.unixtime() / 86400L);
    }
    if (!numericDisplay.begin()) {
        Serial.println(F("Alphanumeric Display did not acknowledge!"));
    } else {
        Serial.println(F("Alphanumeric Display did acknowledge!"));
        numericDisplay->printChar('B', 0);
        numericDisplay->printChar('o', 1);
        numericDisplay->printChar('o', 2);
        numericDisplay->printChar('t', 3);
        numericDisplay->updateDisplay();
    }
    if (!sensor_si7021.begin()) {
        Serial.println(F("Si7021 Sensor not found!"));
    } else {
        Serial.print(F("Found model "));
        switch (sensor_si7021->getModel()) {
            case SI_Engineering_Samples: 
                Serial.print(F("SI engineering samples")); 
                break;
            case SI_7013: 
                Serial.print(F("Si7013")); 
                break;
            case SI_7020: 
                Serial.print(F("Si7020")); 
                break;
            case SI_7021: 
                Serial.print(F("Si7021")); 
                break;
            default: 
                Serial.print(F("Unknown")); 
                break;
        }
        Serial.printf(F(" Rev(%d) Serial #"), sensor_si7021->getRevision());
        Serial.print(sensor_si7021->sernum_a, HEX);
        Serial.println(sensor_si7021->sernum_b, HEX);

        Serial.println();
        Serial.print(F("Humidity:    ")); 
        Serial.print(sensor_si7021->readHumidity(), 2);
        Serial.print(F("\tTemperature:    ")); 
        Serial.println(sensor_si7021->readTemperature(), 2);
    }
    while (true) {
        // do nothing after this point for now
        delay(1000);
    }
    digitalWrite<Pins::RESET, HIGH>();
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
    }
}


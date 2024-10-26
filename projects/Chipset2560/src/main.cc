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
#include <Adafruit_Si7021.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_SPITFT.h>

#include "Types.h"
#include "Pinout.h"
#include "Setup.h"
[[gnu::noinline]] void installInitialBootImage() noexcept;
void configureExternalBus() noexcept;
namespace Pins {
    constexpr auto SD_EN = Pin::PortB0;
    constexpr auto PSRAM_A0 = Pin::PortD2;
    constexpr auto PSRAM_A1 = Pin::PortD3;
    constexpr auto INT960_0 = Pin::PortB4;
    constexpr auto INT960_1 = Pin::PortB5;
    constexpr auto INT960_2 = Pin::PortB6;
    constexpr auto INT960_3 = Pin::PortB7;
    constexpr auto BE1 = Pin::PortD4;
    constexpr auto BE0 = Pin::PortD5;
    constexpr auto WR = Pin::PortD6;
    constexpr auto BLAST = Pin::PortD7;
    constexpr auto RESET = Pin::PortE2;
    constexpr auto HOLD = Pin::PortE3;
    constexpr auto ADS = Pin::PortE4;
    constexpr auto HLDA = Pin::PortE5;
    constexpr auto READY_SYNC_IN = Pin::PortE7;

    constexpr auto PSRAM_EN = Pin::PortG3;
    constexpr auto READY = Pin::PortG5;

}
namespace Ports {
    constexpr auto DataLower = Port::C;
    constexpr auto DataUpper = Port::F;
}

template<typename T>
struct OptionalDevice {
    public:
        template<typename ... Args>
        OptionalDevice(Args ... args) : _device(args...) { }
        constexpr bool valid() const noexcept {
            return _valid;
        }
        auto& get() noexcept {
            return _device;
        }
        template<typename ... Args>
        bool begin(Args&& ... args) noexcept {
            _valid = _device.begin(args...);
            return _valid;
        }
        auto& operator*() const noexcept { return _device; }
        auto* operator->() noexcept { return &_device; }
        const auto* operator->() const noexcept { return &_device; }
        explicit operator bool() const noexcept { return _valid; }
    private:
        bool _valid = false;
        T _device;
};
// use a ds3231 chip
OptionalDevice<RTC_DS3231> rtc;
OptionalDevice<Adafruit_Si7021> sensor_si7021;
OptionalDevice<Adafruit_LTR390> ltr;

class PSRAMBackingStore {
    public:
        using SPIDevice = decltype(SPI);
        PSRAMBackingStore(SPIDevice& link) : _link(link) { }
        void begin() noexcept;
        size_t read(Address addr, uint8_t* storage, size_t count) noexcept;
        size_t write(Address addr, uint8_t* storage, size_t count) noexcept;
        void waitForBackingStoreIdle() noexcept { }
    private:
        void setAddress(Address address) noexcept;
    private:
        SPIDevice& _link;
};

PSRAMBackingStore psramMemory(SPI);
using PSRAMBackingStore = PSRAMBackingStore;
using PSRAMCacheAddress = uint32_t;
using PrimaryBackingStore = PSRAMBackingStore;
using CacheAddress = PSRAMCacheAddress;
#define CommunicationPrimitive psramMemory
using CacheLine = Deception::CacheLine16<CacheAddress, PrimaryBackingStore>;
constexpr auto OnboardCacheLineCount = 256;
using OnboardDataCache = Deception::DirectMappedCache<OnboardCacheLineCount, CacheLine>;
static_assert(sizeof(CacheLine) <= 32);
[[gnu::address(0xFF00)]] volatile CacheLine externalCacheLine;
[[gnu::address(0xFF40)]] volatile struct [[gnu::packed]] {
    struct [[gnu::packed]] {
        uint32_t data;
        uint32_t direction;
    } addressLines;
} interface960;
static_assert(0b1111'1111'0100'0000 == 0xFF40);

class ExternalCacheLineInterface {
    public:
        using Line_t = CacheLine;
        using Address_t = typename Line_t::Address_t;
        using BackingStore_t = typename Line_t::BackingStore_t;
        void begin() noexcept {
            if (!_initialized) {
                _initialized = true;
            }
        }
        static constexpr Address_t normalizeAddress(Address_t address) noexcept {
            return Line_t::normalizeAddress(address);
        }
        static constexpr uint8_t computeOffset(uint8_t input) noexcept {
            return Line_t::computeByteOffset(input);
        }
        void sync(BackingStore_t& store, Address_t address) noexcept {
            // we've already selected the line via hardware acceleration
            auto addr = normalizeAddress(address);
            if (!externalCacheLine.matches(addr)) {
                externalCacheLine.replace(store, addr);
            }
        }
    private:
        bool _initialized = false;
};
using DataCache = ExternalCacheLineInterface;

DataCache cacheInterface;
OnboardDataCache onboardDataCache;

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
        case 0x100: // LTR.available
            if constexpr (readOperation) {
                send32BitConstant(ltr ? 0xFFFF'FFFF : 0x0000'0000);
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x104: // UVS
            if constexpr (readOperation) {
                send32BitConstant(ltr->readUVS());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x108: // ALS
            if constexpr (readOperation) {
                send32BitConstant(ltr->readALS());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x10C: // LTR.Mode
            if constexpr (readOperation) {
                send32BitConstant(ltr->getMode());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x110: // LTR.Gain
            if constexpr (readOperation) {
                send32BitConstant(ltr->getGain());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x114: // LTR.Resolution
            if constexpr (readOperation) {
                send32BitConstant(ltr->getResolution());
            } else {
                doNothingOperation<readOperation>();
            }
            break;
        case 0x118: // LTR.newDataAvailable
            if constexpr (readOperation) {
                send32BitConstant(ltr->newDataAvailable() ? 0xFFFF'FFFF : 0x0000'0000);
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
    cacheInterface.sync(CommunicationPrimitive, address.full);
    auto* ptr = externalCacheLine.getLineData(address.getCacheOffset());
    if constexpr (readOperation) {
        auto ptr16 = reinterpret_cast<volatile uint16_t*>(ptr);
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
            if (upperByteEnabled()) {
                ptr[3] = hi;
            }
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
            if (upperByteEnabled()) {
                ptr[5] = hi;
            }
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
            if (upperByteEnabled()) {
                ptr[7] = hi;
            }
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
            if (upperByteEnabled()) {
                ptr[9] = hi;
            }
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
            if (upperByteEnabled()) {
                ptr[11] = hi;
            }
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
            if (upperByteEnabled()) {
                ptr[13] = hi;
            }
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
        if (upperByteEnabled()) {
            ptr[15] = upperData();
        }
        signalReady<false>();
WriteMemoryDone:
        waitForReady();
    }
}

inline
SplitWord32
getAddress() noexcept {
    return { interface960.addressLines.data };
}

void
configurePins() noexcept {
    pinMode(Pins::RESET, OUTPUT);
    digitalWrite<Pins::RESET, LOW>();
    pinMode(Pins::PSRAM_EN, OUTPUT);
    digitalWrite<Pins::PSRAM_EN, HIGH>();
    pinMode(Pins::PSRAM_A0, OUTPUT);
    digitalWrite<Pins::PSRAM_A0, LOW>();
    pinMode(Pins::PSRAM_A1, OUTPUT);
    digitalWrite<Pins::PSRAM_A1, LOW>();
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
    interface960.addressLines.direction = 0xFFFF'FFFF;
    // then setup the external bus, it is necessary for the next step
}
void sanityCheckHardwareAcceleratedCacheLine() noexcept;
void setupExternalDevices() noexcept;
void configureRandomSeed() noexcept;
static constexpr auto SerialBaudRate = 115200;
void
setup() {
    configureRandomSeed();
    configurePins();
    configureExternalBus();
    configureDataLinesForRead();
    configureInterruptSources();
    Serial.begin(SerialBaudRate);
    Serial.print(F("Serial Up @ "));
    Serial.println(SerialBaudRate);
    SPI.begin();
    Wire.begin();
    Wire.setClock(Deception::TWI_ClockRate);
    cacheInterface.begin();
    onboardDataCache.begin();
    psramMemory.begin();
    installInitialBootImage();
    setupExternalDevices();
    CommunicationPrimitive.waitForBackingStoreIdle();
    sanityCheckHardwareAcceleratedCacheLine();
    // do not cache anything to start with as we should instead just be ready
    // to get data from psram instead
    interface960.addressLines.direction = 0;
    digitalWrite<Pins::RESET, HIGH>();
}
void
setupExternalDevices() noexcept {
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

    if (!ltr.begin()) {
        Serial.println(F("Couldn't find LTR sensor!"));
    } else {
        Serial.println(F("Found LTR sensor!"));

        ltr->setMode(LTR390_MODE_UVS);
        if (ltr->getMode() == LTR390_MODE_ALS) {
            Serial.println(F("In ALS mode"));
        } else {
            Serial.println(F("In UVS mode"));
        }

        ltr->setGain(LTR390_GAIN_3);
        Serial.print(F("Gain : "));
        switch (ltr->getGain()) {
            case LTR390_GAIN_1: 
                Serial.println(1); 
                break;
            case LTR390_GAIN_3: 
                Serial.println(3); 
                break;
            case LTR390_GAIN_6: 
                Serial.println(6); 
                break;
            case LTR390_GAIN_9: 
                Serial.println(9); 
                break;
            case LTR390_GAIN_18: 
                Serial.println(18); 
                break;
            default:
                Serial.println('?');
                break;
        }

        ltr->setThresholds(100, 1000);
        ltr->configInterrupt(true, LTR390_MODE_UVS);
        while (!ltr->newDataAvailable()) {
            delay(100);
        }
        Serial.printf(F("UV data: %d\n"), ltr->readUVS());
    }
}
inline void 
processMemoryRequest() noexcept {
    // clear the READY signal interrupt ahead of waiting for the last
    clearREADYInterrupt();
    do { } while (bit_is_clear(EIFR, ADSFLAG));
    clearADSInterrupt();
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
void 
loop() {
    processMemoryRequest();
}


void
PSRAMBackingStore::begin() noexcept {
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
    _link.beginTransaction(SPISettings{5'000'000, MSBFIRST, SPI_MODE0});
    digitalWrite<Pins::PSRAM_A0, LOW>();
    digitalWrite<Pins::PSRAM_A1, LOW>();
    activatePSRAM(true, true);
    digitalWrite<Pins::PSRAM_A0, HIGH>();
    digitalWrite<Pins::PSRAM_A1, LOW>();
    activatePSRAM(true, true);
    digitalWrite<Pins::PSRAM_A0, LOW>();
    digitalWrite<Pins::PSRAM_A1, HIGH>();
    activatePSRAM(true, true);
    digitalWrite<Pins::PSRAM_A0, HIGH>();
    digitalWrite<Pins::PSRAM_A1, HIGH>();
    activatePSRAM(true, true);
    // we need to go through and enable a
    _link.endTransaction();
    digitalWrite<Pins::PSRAM_A0, LOW>();
    digitalWrite<Pins::PSRAM_A1, LOW>();

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

void 
PSRAMBackingStore::setAddress(Address address) noexcept {
    uint8_t addr = static_cast<uint8_t>(address >> 23) & 0b11;
    if (addr & 0b10) {
        digitalWrite<Pins::PSRAM_A1, HIGH>();
    } else {
        digitalWrite<Pins::PSRAM_A1, LOW>();
    }
    if (addr & 0b01) {
        digitalWrite<Pins::PSRAM_A0, HIGH>();
    } else {
        digitalWrite<Pins::PSRAM_A0, LOW>();
    }
}
size_t
PSRAMBackingStore::read(Address addr, uint8_t* storage, size_t count) noexcept {
    setAddress(addr);
    _link.beginTransaction(SPISettings{5'000'000, MSBFIRST, SPI_MODE0});
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

size_t
PSRAMBackingStore::write(Address addr, uint8_t* storage, size_t count) noexcept {
    setAddress(addr);
    _link.beginTransaction(SPISettings{5'000'000, MSBFIRST, SPI_MODE0});
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
    Serial.println(F("Zeroing out cache memory"));
    for (uint32_t i = 0; i < (1024ul * 1024ul); i += 16) {
        interface960.addressLines.data = i;
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
    bitSet(XMCRA, SRW11);
    bitClear(XMCRA, SRW10);
    bitSet(XMCRA, SRW01);
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
    bitSet(XMCRB, XMBK); // yes bus keeper
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


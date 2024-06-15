/*
i960SxChipset_Type103
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
#include <RTClib.h>


#include "Detect.h"
#include "Types.h"
#include "Pinout.h"
#include "Setup.h"

// Each AVR cycle takes some amount of time to complete. In this case, it is
// 50ns. When we use the 20MHz clock directly in the i960 then we have a cycle
// time of 100ns or two avr clock cycles. If we feed a 10MHz signal in the i960
// then we get 4 clocks per i960 cycle.
//
constexpr uint32_t MicrocontrollerSpeed = F_CPU;
constexpr uint32_t HalfClockSpeed = F_CPU / 2;
constexpr uint32_t QuarterClockSpeed = F_CPU / 4;
constexpr uint32_t CPUSpeed = QuarterClockSpeed;
constexpr uint32_t NumberOfMicrocontrollerClocksPerCPUClock = MicrocontrollerSpeed / CPUSpeed;

// Given the number of i960 clocks we return the equivalent number of AVR
// cycles
constexpr uint32_t operator "" _clocks_i960(unsigned long long value) {
    return static_cast<uint32_t>(value) * NumberOfMicrocontrollerClocksPerCPUClock;
}

static_assert(3_clocks_i960 == (3 * NumberOfMicrocontrollerClocksPerCPUClock));
static_assert(4_clocks_i960 == (4 * NumberOfMicrocontrollerClocksPerCPUClock));

extern "C" [[noreturn]] void ExecutionBody();
extern "C" [[noreturn]] void ExecutionBody_5MHz();
extern "C" [[gnu::used]] void doIOReadOperation();
extern "C" [[gnu::used]] void doIOWriteOperation();
using DataRegister8 = volatile uint8_t*;
using DataRegister16 = volatile uint16_t*;
SdFs SD;
FsFile disk0;
RTC_PCF8523 rtc;
constexpr auto MaximumBootImageFileSize = 1024ul * 1024ul;
constexpr bool PerformMemoryImageInstallation = true;

[[gnu::always_inline]] inline bool isBurstLast() noexcept { 
    return digitalRead<Pin::BLAST>() == LOW; 
}

template<auto ViewKind>
struct ViewIs final {
    using Type = decltype(ViewKind);
    static constexpr Type Value = ViewKind;
};

struct DataPortInterface {
    using Self = DataPortInterface;
    DataPortInterface() = delete;
    ~DataPortInterface() = delete;
    DataPortInterface(const Self&) = delete;
    DataPortInterface(Self&&) = delete;
    Self& operator=(const Self&) = delete;
    Self& operator=(Self&&) = delete;
    static void setLowerDataLinesDirection(uint8_t value) noexcept {
        getDirectionRegister<Port::DataLinesLower>() = value;
    }
    static void setUpperDataLinesDirection(uint8_t value) noexcept {
        getDirectionRegister<Port::DataLinesUpper>() = value;
    }
    static inline uint8_t getLowerDataByte() noexcept { return getInputRegister<Port::DataLinesLower>(); }
    static inline uint8_t getUpperDataByte() noexcept { return getInputRegister<Port::DataLinesUpper>(); }
    static inline void setLowerDataByte(uint8_t value) noexcept { getOutputRegister<Port::DataLinesLower>() = value; }
    static inline void setUpperDataByte(uint8_t value) noexcept { getOutputRegister<Port::DataLinesUpper>() = value; }
    static void configureInterface() noexcept {
        getDirectionRegister<Port::DataLinesUpper>() = 0xff;
        getDirectionRegister<Port::DataLinesLower>() = 0xff;
    }
};




struct DataInterface {
    using UnderlyingDataInterface = DataPortInterface;
    using Self = DataInterface;
    DataInterface() = delete;
    ~DataInterface() = delete;
    DataInterface(const Self&) = delete;
    DataInterface(Self&&) = delete;
    Self& operator=(const Self&) = delete;
    Self& operator=(Self&&) = delete;
    static void configureInterface() noexcept {
        UnderlyingDataInterface::configureInterface();
    }
    static inline uint16_t getData() noexcept {
        return makeWord(UnderlyingDataInterface::getUpperDataByte(), UnderlyingDataInterface::getLowerDataByte());
    }
    static inline void setData(uint16_t value) noexcept {
        UnderlyingDataInterface::setLowerDataByte(value);
        UnderlyingDataInterface::setUpperDataByte(static_cast<uint8_t>(value >> 8));
    }
    static void setLowerDataLinesDirection(uint8_t value) noexcept {
        UnderlyingDataInterface::setLowerDataLinesDirection(value);
    }
    static void setUpperDataLinesDirection(uint8_t value) noexcept {
        UnderlyingDataInterface::setUpperDataLinesDirection(value);
    }
    static uint8_t getLowerDataByte() noexcept { 
        return UnderlyingDataInterface::getLowerDataByte();
    }
    static uint8_t getUpperDataByte() noexcept { 
        return UnderlyingDataInterface::getUpperDataByte();
    }
    static void setLowerDataByte(uint8_t value) noexcept { 
        UnderlyingDataInterface::setLowerDataByte(value);
    }
    static void setUpperDataByte(uint8_t value) noexcept { 
        UnderlyingDataInterface::setUpperDataByte(value);
    }
};

template<uint8_t delayAmount = 2_clocks_i960>
[[gnu::always_inline]] 
inline 
void 
signalReady() noexcept {
    toggle<Pin::ReadyDirect>();
    if constexpr (delayAmount > 0) {
        // it takes two cycles avr for the ready signal to actually signal
        // ready. The i960 needs to detect the signal and act accordingly.
        // so it will be 6 cycles every single time between the time we start
        // the trigger and when we have the next transaction starting
        insertCustomNopCount<delayAmount + 2_clocks_i960>();
    }
}

using Register8 = volatile uint8_t&;
using Register16 = volatile uint16_t&;
template<int index>
struct TimerDescriptor { 
};
#define X(index) \
template<> \
struct TimerDescriptor< index > {  \
    static inline Register8 TCCRxA = TCCR ## index ## A ; \
    static inline Register8 TCCRxB = TCCR ## index ## B ; \
    static inline Register8 TCCRxC = TCCR ## index ## C ; \
    static inline Register16 TCNTx = TCNT ## index; \
    static inline Register16 ICRx = ICR ## index ; \
    static inline Register16 OCRxA = OCR ## index ## A ; \
    static inline Register16 OCRxB = OCR ## index ## B ; \
    static inline Register16 OCRxC = OCR ## index ## C ; \
}; \
constexpr TimerDescriptor< index > timer ## index 
X(1);
#undef X


template<uint8_t index>
inline void setDataByte(uint8_t value) noexcept {
    static_assert(index < 2, "Invalid index provided to setDataByte, must be less than 2");
    if constexpr (index < 2) {
        if constexpr (index == 0) {
            DataInterface::setLowerDataByte(value);
        } else {
            DataInterface::setUpperDataByte(value);
        }
    }
}

template<uint8_t index>
inline uint8_t getDataByte() noexcept {
    static_assert(index < 2, "Invalid index provided to getDataByte, must be less than 4");
    if constexpr (index < 2) {
        if constexpr (index == 0) {
            return DataInterface::getLowerDataByte();
        } else {
            return DataInterface::getUpperDataByte();
        }
    } else {
        return 0;
    }
}


struct MemoryInterface {
    using Self = MemoryInterface;
    MemoryInterface() = delete;
    ~MemoryInterface() = delete;
    MemoryInterface(const Self&) = delete;
    MemoryInterface(Self&&) = delete;
    Self& operator=(const Self&) = delete;
    Self& operator=(Self&&) = delete;
private:
    static constexpr uint8_t MemoryWindowUpperHalf = 0xFC;
    static constexpr uintptr_t MemoryWindowBaseAddress = (static_cast<uint16_t>(MemoryWindowUpperHalf) << 8);
    static_assert(MemoryWindowBaseAddress == 0xFC00);
    static constexpr auto TransferBufferSize = 256;
    [[gnu::noinline]] static void displayMemoryBody(volatile uint8_t* body, int size = TransferBufferSize) noexcept {
        for (int i = 0; i < size; ++i) {
            auto value = body[i];
            Serial.printf(F("@0x%x: 0x%x\n"), i, value);
        }
    }
    static inline void setUpper24AddressLines(uint32_t address) noexcept {
        getOutputRegister<AddressLines[1]>() = static_cast<uint8_t>(address >> 8);
        getOutputRegister<AddressLines[2]>() = static_cast<uint8_t>(address >> 16);
        getOutputRegister<AddressLines[3]>() = static_cast<uint8_t>(address >> 24);
    }
    [[gnu::noinline]] static void displayMemoryBody(uint32_t address, volatile uint8_t* body, int size = TransferBufferSize) noexcept {
        setUpper24AddressLines(address);
        displayMemoryBody(body, size);
    }
    template<bool printInBinary = false>
    static bool sanityCheck(uint8_t* buf0, uint8_t* buf1, uint32_t address, int size, bool printAnyway = false) noexcept {
        bool mismatch = false;
        for (int i = 0; i < size; ++i) {
            auto a = buf0[i];
            auto b = buf1[i];
            if (a != b) {
                if constexpr (printInBinary) {
                    Serial.printf(F("Mismatch@0x%lx want: 0b"), address + i);
                    Serial.print(a, BIN);
                    Serial.print(F(" got: 0b"));
                    Serial.println(b, BIN);
                } else {
                    Serial.printf(F("Mismatch@0x%lx want: 0x%x got: 0x%x\n"), address + i, a, b);
                }
                mismatch = true;
            } else {
                if (printAnyway || mismatch) {
                    if constexpr (printInBinary) {
                        Serial.printf(F("Match@0x%lx want: 0b"), address + i);
                        Serial.print(a, BIN);
                        Serial.print(F(" got: 0b"));
                        Serial.println(b, BIN);
                    } else { 
                        Serial.printf(F("Match@0x%lx want: 0x%x got: 0x%x\n"), address + i, a, b);
                    }
                }
            }
        }
        return mismatch;
    }
public:
    static void configure() noexcept {
    }
    [[gnu::noinline]]
    static void installMemoryImage(File& theFirmware) {
        uint8_t temporaryBuffer[256];
        uint8_t temporaryBuffer2[256];
        volatile auto* theBuffer = memoryPointer<uint8_t>(MemoryWindowBaseAddress);
        int counter = 0;
        for (uint32_t address = 0; address < theFirmware.size(); address += TransferBufferSize, ++counter) {
            // just modify the bank as we go along
            setUpper24AddressLines(address);
            theFirmware.read(temporaryBuffer, TransferBufferSize);
            memcpy(const_cast<uint8_t*>(theBuffer), temporaryBuffer, TransferBufferSize); 
            memcpy(temporaryBuffer2, const_cast<uint8_t*>(theBuffer), TransferBufferSize);
            if (sanityCheck(temporaryBuffer, temporaryBuffer2, address, TransferBufferSize)) {
                // halt here
                while (true);
            }
            if ((counter % 32) == 0) {
                Serial.print('.');
            }
        }
        {
            // clear temporaryBuffer2 for a second sanity check
            Serial.println(F("Sanity Check"));
            Serial.print(F("HLDA State: "));
            if (digitalRead<Pin::HLDA>() == HIGH) {
                Serial.println(F("HIGH"));
            } else {
                Serial.println(F("LOW"));
            }
            theFirmware.seekSet(0);
            setUpper24AddressLines(0);
            memset(temporaryBuffer, 0, TransferBufferSize);
            memset(temporaryBuffer2, 0, TransferBufferSize);
            theFirmware.read(temporaryBuffer, TransferBufferSize);
            memcpy(temporaryBuffer2, const_cast<uint8_t*>(theBuffer), TransferBufferSize);
            if (sanityCheck<true>(temporaryBuffer, temporaryBuffer2, 0, TransferBufferSize)) {
                // okay, so just run through memory starting from here
#if 0
                for (uint32_t address = 0x100; address < theFirmware.size(); address += TransferBufferSize) {
                    setUpper24AddressLines(address);
                    theFirmware.read(temporaryBuffer, TransferBufferSize);
                    memcpy(temporaryBuffer2, const_cast<uint8_t*>(theBuffer), TransferBufferSize);
                    for (int i = 0; i < TransferBufferSize; ++i) {
                        auto a = temporaryBuffer[i];
                        auto b = temporaryBuffer2[i];
                        if (a != b) {
                            Serial.printf(F("Mismatch@0x%lx want: 0x%x got: 0x%x\n"), address + i, a, b);
                        }
                    }
                }
#endif
                while (true);
            }


        }
    }
};

#define I960_Signal_Switch \
    if (isBurstLast()) { \
        break; \
    } \
    signalReady()
template<bool isReadOperation>
inline
void
doNothing() {
    if constexpr (isReadOperation) {
        DataInterface::setData(0);
    }
    while (!isBurstLast()) {
        signalReady();
    }
    signalReady<0>();
}
template<bool isReadOperation>
inline 
void
doLegacyIO() noexcept {
    switch ( getInputRegister<AddressLines[0]>()) {
    
        case 0: { 
                    if constexpr (isReadOperation) { 
                        DataInterface::setData(static_cast<uint16_t>(MicrocontrollerSpeed));
                    } 
                    I960_Signal_Switch;
                } 
        case 2: { 
                    if constexpr (isReadOperation) { 
                        DataInterface::setData(static_cast<uint16_t>((MicrocontrollerSpeed) >> 16));
                    } 
                    I960_Signal_Switch;
                } 
        case 4: { 
                    if constexpr (isReadOperation) { 
                        DataInterface::setData(static_cast<uint16_t>(CPUSpeed));
                    } 
                    I960_Signal_Switch;
                } 
        case 6: { 
                    if constexpr (isReadOperation) { 
                        DataInterface::setData(static_cast<uint16_t>((CPUSpeed) >> 16));
                    } 
                    I960_Signal_Switch;
                } 
        case 8: { 
                    /* Serial RW connection */
                    if constexpr (isReadOperation) { 
                        DataInterface::setData(Serial.read());
                    } else { 
                        // no need to check this out just ignore the byte
                        // enable lines
                        // needs to be separate operations to make sure that
                        // the compiler doesn't do dumb things when optimizing!
                        volatile uint8_t value = getDataByte<0>();
                        Serial.write(value);
                    } 
                    I960_Signal_Switch;
                } 
        case 10: {
                     if constexpr (isReadOperation) { 
                         DataInterface::setData(0);
                     } 
                     I960_Signal_Switch;
                 } 
        case 12: { 
                     if constexpr (isReadOperation) { 
                         DataInterface::setData(0);
                     } else { 
                         Serial.flush();
                     }
                     I960_Signal_Switch;
                 }
        case 14: {
                     /* nothing to do on writes but do update the data port
                      * on reads */ 
                     if constexpr (isReadOperation) { 
                         DataInterface::setData(0);
                     } 
                 }
                 break;
#define X(obj, offset) \
        case offset + 0: { \
                             /* TCCRnA and TCCRnB */ \
                             if constexpr (isReadOperation) { \
                                 setDataByte<0>(obj.TCCRxA);\
                                 setDataByte<1>(obj.TCCRxB);\
                             } else { \
                                 if (digitalRead<Pin::BE0>() == LOW) { \
                                     obj.TCCRxA = getDataByte<0>();\
                                 } \
                                 if (digitalRead<Pin::BE1>() == LOW) { \
                                     obj.TCCRxB = getDataByte<1>();\
                                 } \
                             } \
                             I960_Signal_Switch;\
                         } \
        case offset + 2: { \
                             /* TCCRnC and Reserved (ignore that) */ \
                             if constexpr (isReadOperation) { \
                                 setDataByte<0>(obj.TCCRxC);\
                                 setDataByte<1>(0); \
                             } else { \
                                 if (digitalRead<Pin::BE0>() == LOW) { \
                                     obj.TCCRxC = getDataByte<0>();\
                                 } \
                             } \
                             I960_Signal_Switch;\
                         } \
        case offset + 4: { \
                             /* TCNTn should only be accessible if you do a full 16-bit
                              * write 
                              */ \
                             if constexpr (isReadOperation) { \
                                 noInterrupts(); \
                                     auto tmp = obj.TCNTx; \
                                     interrupts();  \
                                     DataInterface::setData(tmp); \
                             } else {  \
                                 if (digitalRead<Pin::BE0>() == LOW &&  \
                                         digitalRead<Pin::BE1>() == LOW) {  \
                                     auto value = DataInterface::getData();  \
                                         noInterrupts();  \
                                         obj.TCNTx = value; \
                                         interrupts();  \
                                 }  \
                             }  \
                             I960_Signal_Switch; \
                         }  \
        case offset + 6: {  \
                             /* ICRn should only be accessible if you do a full 16-bit 
                              * write
                              */  \
                             if constexpr (isReadOperation) { \
                                 noInterrupts(); \
                                     auto tmp = obj.ICRx;\
                                     interrupts(); \
                                     DataInterface::setData(tmp); \
                             } else { \
                                 if (digitalRead<Pin::BE0>() == LOW &&  \
                                         digitalRead<Pin::BE1>() == LOW) { \
                                     auto value = DataInterface::getData(); \
                                         noInterrupts(); \
                                         obj.ICRx = value;\
                                         interrupts(); \
                                 } \
                             } \
                             I960_Signal_Switch;\
                         } \
        case offset + 8: { \
                             /* OCRnA should only be accessible if you do a full 16-bit write */ \
                                 if constexpr (isReadOperation) { \
                                     noInterrupts(); \
                                         auto tmp = obj.OCRxA;\
                                         interrupts(); \
                                         DataInterface::setData(tmp); \
                                 } else { \
                                     if (digitalRead<Pin::BE0>() == LOW &&  \
                                             digitalRead<Pin::BE1>() == LOW) { \
                                         auto value = DataInterface::getData(); \
                                             noInterrupts(); \
                                             obj.OCRxA = value;\
                                             interrupts(); \
                                     } \
                                 } \
                             I960_Signal_Switch;\
                         } \
        case offset + 10: {\
                              /* OCRnB */ \
                                  if constexpr (isReadOperation) { \
                                      noInterrupts(); \
                                          auto tmp = obj.OCRxB;\
                                          interrupts(); \
                                          DataInterface::setData(tmp); \
                                  } else { \
                                      if (digitalRead<Pin::BE0>() == LOW &&  \
                                              digitalRead<Pin::BE1>() == LOW) { \
                                          auto value = DataInterface::getData(); \
                                              noInterrupts(); \
                                              obj.OCRxB = value; \
                                              interrupts(); \
                                      } \
                                  } \
                              I960_Signal_Switch;\
                          } \
        case offset + 12: { \
                              /* OCRnC */ \
                                  if constexpr (isReadOperation) { \
                                      noInterrupts(); \
                                          auto tmp = obj.OCRxC; \
                                          interrupts(); \
                                          DataInterface::setData(tmp); \
                                  } else { \
                                      if (digitalRead<Pin::BE0>() == LOW && \
                                              digitalRead<Pin::BE1>() == LOW) { \
                                          auto value = DataInterface::getData(); \
                                              noInterrupts(); \
                                              obj.OCRxC = value;\
                                              interrupts(); \
                                      }\
                                  } \
                              I960_Signal_Switch;\
                          } \
        case offset + 14: { \
                              /* nothing to do on writes but do update the data port
                               * on reads */ \
                              if constexpr (isReadOperation) { \
                                  DataInterface::setData(0); \
                              } \
                              break;\
                          }  
#ifdef TCCR1A
                          X(timer1, 0x10);
#endif
#undef X
        case 0x40: {
                       if constexpr (isReadOperation) {
                           volatile uint32_t result = millis();
                           DataInterface::setData(static_cast<uint16_t>(result));
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(result >> 16));
                           I960_Signal_Switch;
                       } else {
                           I960_Signal_Switch;
                           I960_Signal_Switch;
                       }
                   }
        case 0x44: {
                       if constexpr (isReadOperation) {
                           volatile uint32_t result = micros();
                           DataInterface::setData(static_cast<uint16_t>(result));
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(result >> 16));
                           I960_Signal_Switch;
                       } else {
                           I960_Signal_Switch;
                           I960_Signal_Switch;
                       }
                   }
        case 0x48: {
                       if constexpr (isReadOperation) {
                           DataInterface::setData(0);
                       }
                       I960_Signal_Switch;
                       I960_Signal_Switch;
                   }
        case 0x4c: {
                       if constexpr (isReadOperation) {
                           DataInterface::setData(0);
                       }
                       I960_Signal_Switch;
                       I960_Signal_Switch;
                   }
                   break;
                   // disk0 is special and is supposed to be as available as
                   // possible
#if 0
                   struct Disk0View {
                       uint64_t capacity; // in bytes
                       uint64_t position; // in bytes
                       uint16_t available; // 
                       int16_t rw; 
                   };
#endif
        case 0x50: {
                       // capacity register
                       if constexpr (isReadOperation) {
                           auto capacity = disk0.size();
                           DataInterface::setData(capacity);
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(capacity >> 16));
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(capacity >> 32));
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(capacity >> 48));
                           I960_Signal_Switch;
                       } else {
                           I960_Signal_Switch;
                           I960_Signal_Switch;
                           I960_Signal_Switch;
                           I960_Signal_Switch;
                       }
                   }
        case 0x58: {
                       // setup the seeking register
                       if constexpr (isReadOperation) {
                           auto pos = disk0.curPosition();
                           DataInterface::setData(pos);
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(pos >> 16));
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(pos >> 32));
                           I960_Signal_Switch;
                           DataInterface::setData(static_cast<uint16_t>(pos >> 48));
                           I960_Signal_Switch;
                       } else {
                           uint64_t a = static_cast<uint64_t>(DataInterface::getData());
                           I960_Signal_Switch;
                           uint64_t b = static_cast<uint64_t>(DataInterface::getData()) << 16;
                           I960_Signal_Switch;
                           uint64_t c = static_cast<uint64_t>(DataInterface::getData()) << 32;
                           I960_Signal_Switch;
                           uint64_t d = static_cast<uint64_t>(DataInterface::getData()) << 48;
                           uint64_t newPosition = a | b | c | d;
                           if (newPosition >= disk0.size()) {
                               newPosition %= disk0.size();
                           }
                           disk0.seek(newPosition);
                           I960_Signal_Switch;
                       }

                   }
        case 0x60: {
                       // available lookup
                       if constexpr (isReadOperation) {
                           DataInterface::setData(disk0.isOpen() ? 0xFFFF : 0);
                       } 
                       I960_Signal_Switch;
                   }
        case 0x62: {
                       // rw port
                       if constexpr (isReadOperation) {
                           int16_t result = disk0.read();
                           DataInterface::setData(result);
                       } else {
                           // write a value out
                           auto value = static_cast<uint8_t>(DataInterface::getData());
                           (void)disk0.write(value);
                       }
                       I960_Signal_Switch;
                   }
        default:
                   doNothing<isReadOperation>();
                   return;
    } 
    signalReady<0>(); 
}
class DataBlock {
public:
    void clear() {
        for (int i = 0 ; i< 256; ++i) {
            block[i] = 0;
        }
    }
    template<bool isReadOperation>
    void processRequest() {
        DataRegister8 blockStart = &block[getInputRegister<Port::AddressLinesLowest>()];
        if constexpr (isReadOperation) {
            processReadRequest(blockStart);
        } else {
            processWriteRequest(blockStart);
        }
    }
    [[nodiscard]] uint8_t& operator[](uint8_t index) noexcept {
        return block[index];
    }

private:
    void 
    processReadRequest(DataRegister8 ptr) noexcept {
        DataRegister16 ptr16 = reinterpret_cast<DataRegister16>(ptr);
        DataInterface::setData(ptr16[0]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[1]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[2]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[3]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[4]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[5]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[6]);
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        DataInterface::setData(ptr16[7]);
        signalReady();
    }
    void 
    processWriteRequest(DataRegister8 ptr) noexcept {
        if (digitalRead<Pin::BE0>() == LOW) {
            ptr[0] = getDataByte<0>();
        }
        if (digitalRead<Pin::BE1>() == LOW) {
            ptr[1] = getDataByte<1>();
        }
        if (isBurstLast()) {
            signalReady<0>();
            return;
        }
        signalReady();
        // flow into this
        ptr[2] = getDataByte<0>();
        if (isBurstLast()) {
            if (digitalRead<Pin::BE1>() == LOW) {
                ptr[3] = getDataByte<1>();
            }
            signalReady<0>();
            return;
        }
        ptr[3] = getDataByte<1>();
        signalReady();
        // flow into this
        ptr[4] = getDataByte<0>();
        if (isBurstLast()) {
            if (digitalRead<Pin::BE1>() == LOW) {
                ptr[5] = getDataByte<1>();
            }
            signalReady<0>();
            return;
        }
        ptr[5] = getDataByte<1>();
        signalReady();
        // we flow into this
        ptr[6] = getDataByte<0>();
        if (isBurstLast()) {
            if (digitalRead<Pin::BE1>() == LOW) {
                ptr[7] = getDataByte<1>();
            }
            signalReady<0>();
            return;
        }
        ptr[7] = getDataByte<1>();
        signalReady();
        // we flow into this
        ptr[8] = getDataByte<0>();
        if (isBurstLast()) {
            if (digitalRead<Pin::BE1>() == LOW) {
                ptr[9] = getDataByte<1>();
            }
            signalReady<0>();
            return;
        }
        ptr[9] = getDataByte<1>();
        signalReady();
        // we flow into this
        ptr[10] = getDataByte<0>();
        if (isBurstLast()) {
            if (digitalRead<Pin::BE1>() == LOW) {
                ptr[11] = getDataByte<1>();
            }
            signalReady<0>();
            return;
        }
        ptr[11] = getDataByte<1>();
        signalReady();
        // we flow into this
        ptr[12] = getDataByte<0>();
        if (isBurstLast()) {
            if (digitalRead<Pin::BE1>() == LOW) {
                ptr[13] = getDataByte<1>();
            }
            signalReady<0>();
            return;
        }
        ptr[13] = getDataByte<1>();
        signalReady();
        ptr[14] = getDataByte<0>();
        if (digitalRead<Pin::BE1>() == LOW) {
            ptr[15] = getDataByte<1>();
        }
        signalReady();
    }
private:
    uint8_t block[256];
};

constexpr auto NumberOfBlocks = 4;
DataBlock blocks[NumberOfBlocks];
void 
setupDataBlocks() {
    for (int i = 0; i < NumberOfBlocks; ++i) {
        for (int j = 0; j < 256; ++j) {
            blocks[i][j] = random();
        }
        blocks[i].clear();
    }
}
template<bool isReadOperation>
void 
doIO() noexcept { 
    switch (getInputRegister<AddressLines[1]>()) {
        case 0x00:
            doLegacyIO<isReadOperation>();
            break;
        case 0x01: // this is a 256byte data mapping
            blocks[0].processRequest<isReadOperation>();
            break;
        case 0x02: // this is a 256byte data mapping
            blocks[1].processRequest<isReadOperation>();
            break;
        case 0x03: // this is a 256byte data mapping
            blocks[2].processRequest<isReadOperation>();
            break;
        case 0x04: // this is a 256byte data mapping
            blocks[3].processRequest<isReadOperation>();
            break;
        default:
            doLegacyIO<isReadOperation>();
            break;
    }
}

void 
doIOReadOperation() {
    doIO<true>();
}
void 
doIOWriteOperation() {
    doIO<false>();
}
#undef I960_Signal_Switch

template<uint32_t maxFileSize = MaximumBootImageFileSize>
[[gnu::noinline]]
void
installMemoryImage() noexcept {
    static constexpr uint32_t MaximumFileSize = maxFileSize;
    SPI.beginTransaction(SPISettings(F_CPU / 2, MSBFIRST, SPI_MODE0)); // force to 10 MHz
#define filePath (F("prog.bin"))

    //static constexpr auto filePath = "prog.bin";
    // look for firmware.bin and open it readonly
    if (auto theFirmware = SD.open(filePath, FILE_READ); !theFirmware) {
        Serial.printf(F("Could not open %s for reading!"), filePath);
        while (true) {
            delay(1000);
        }
    } else if (theFirmware.size() > MaximumFileSize) {
        Serial.println(F("The firmware image is too large to fit in sram!"));
        while (true) {
            delay(1000);
        }
    } else {
        Serial.println(F("TRANSFERRING!!"));
        MemoryInterface::installMemoryImage(theFirmware);
        Serial.println(F("DONE!"));
        theFirmware.close();
    }
    // okay so now end reading from the SD Card
    SPI.endTransaction();
#undef filePath
}
template<Pin p>
[[gnu::always_inline]]
inline
void
outputPin() noexcept {
    pinMode(p, OUTPUT);
}

template<Pin p>
[[gnu::always_inline]]
inline
void
inputPin() noexcept {
    pinMode(p, INPUT);
}



void
configurePins() noexcept {
    // put the i960 in reset as soon as possible to make sure
    outputPin<Pin::Reset>();
    digitalWrite<Pin::Reset, LOW>();
    outputPin<Pin::XINT0>();
    digitalWrite<Pin::XINT0, HIGH>();
    outputPin<Pin::XINT2>();
    digitalWrite<Pin::XINT2, LOW>();
    outputPin<Pin::XINT4>();
    digitalWrite<Pin::XINT4, LOW>();
    outputPin<Pin::XINT6>();
    digitalWrite<Pin::XINT6, HIGH>();
    outputPin<Pin::ReadyDirect>();
    digitalWrite<Pin::ReadyDirect, HIGH>();
    outputPin<Pin::Hold>();
    digitalWrite<Pin::Hold, LOW>();

    inputPin<Pin::Lock>();
    inputPin<Pin::WR>();
    inputPin<Pin::IsIOOperation>();
    inputPin<Pin::ADS>();
    inputPin<Pin::HLDA>();
    inputPin<Pin::BE0>();
    inputPin<Pin::BE1>();
    inputPin<Pin::BLAST>();
}
void
configureRTC() noexcept {
    if (!rtc.begin()) {
        Serial.println(F("NO RTC Found!"));
        bitClear(GPIOR0, 7);
    } else {
        bitSet(GPIOR0, 7);
        if (!rtc.initialized() || rtc.lostPower()) {
            Serial.println(F("RTC is NOT initialized, performing default configuration!"));

            // set the RTC to the compilation time of this firmware
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        rtc.start();
        // default configuration values found in the example sketch
        // pcf8523.ino, it seems to work pretty well in all the cases I've
        // found
        float drift = 43; // seconds plus or minus over the observation period
                          // - set to 0 to cancel previous calibration
        float periodSeconds = (7 * 86400); // total observation period in
                                           // seconds (86400 is the number of
                                           // seconds in 1 day: 7 days * 86400)
        float deviationPPM = (drift/periodSeconds * 1'000'000); // deviation in
                                                             // parts per
                                                             // million
        float driftUnit = 4.34; // use with offset mode PCF8523_TwoHours
        int offset = round(deviationPPM / driftUnit);
        rtc.calibrate(PCF8523_TwoHours, offset); // perform calibration every
                                                 // two hours
        DateTime now = rtc.now();
        Serial.printf(F("Current rtc unixtime is: %ld\n"), now.unixtime());
    }
}
void
setup() {
    GPIOR0 = 0;
    // now we can start setting up peripherals
    Serial.begin(115200);
    Wire.begin();
    SPI.begin();
    // power down the ADC
    // currently we can't use them
    PRR0 = 0b0000'0001; // deactivate ADC
    PRR1 = 0b0000'0110; // deactivate USART2 and USART3, they are not
                        // accessible at all!
                        // But leave the timers available since they can be
                        // used for internal purposes
    configurePins();
    configureRTC();

    // setup the EBI
    XMCRB=0b1'0000'111;
    XMCRA=0b1'010'11'11;  
    // we divide the sector limits so that it 0x2200-0x3FFF and 0x4000-0xFFFF
    // the single cycle wait state is necessary even with the AHC573s
    DataInterface::configureInterface();
    MemoryInterface::configure();

    getDirectionRegister<AddressLines[0]>() = 0;
    getOutputRegister<AddressLines[0]>() = 0;
    getDirectionRegister<AddressLines[1]>() = 0xFF; // just configure them all for output
    getOutputRegister<AddressLines[1]>() = 0;
    getDirectionRegister<AddressLines[2]>() = 0xFF; // just configure them all for output
    getOutputRegister<AddressLines[2]>() = 0;
    getDirectionRegister<AddressLines[3]>() = 0xFF;
    getOutputRegister<AddressLines[3]>() = 0;
    setupDataBlocks();
    Serial.println(F("Looking for an SD Card!"));
    {
        while (!SD.begin(static_cast<int>(Pin::SD_EN))) {
            Serial.println(F("NO SD CARD!"));
            delay(1000);
        }
    }
    Serial.println(F("SD CARD FOUND!"));
    // find firmware.bin and install it into the 512k block of memory
    if constexpr (PerformMemoryImageInstallation) {
        installMemoryImage();
    } else {
        delay(1000);
    }
    disk0 = SD.open("disk0.dsk", O_RDWR);
    if (!disk0) {
        Serial.println(F("Could not open disk0.dsk"));
        Serial.println(F("No hard drive will be available"));
    }
    getDirectionRegister<AddressLines[1]>() = 0; 
    getOutputRegister<AddressLines[1]>() = 0; 
    getDirectionRegister<AddressLines[2]>() = 0; 
    getOutputRegister<AddressLines[2]>() = 0;
    getDirectionRegister<AddressLines[3]>() = 0;
    getOutputRegister<AddressLines[3]>() = 0;
    // attach interrupts
    if constexpr (CPUSpeed == HalfClockSpeed) {
        EICRA = 0b0010'0000; // falling edge on INT2 only
    } else if constexpr (CPUSpeed == QuarterClockSpeed) {
        EICRA = 0b0011'0000; // rising edge on INT2 only
    }
    digitalWrite<Pin::Reset, HIGH>(); 
    digitalWrite<Pin::Hold, LOW>();
}
void 
loop() {
    // instead of controlling execution directly, we should be using a slight
    // time slicing design to make sure that we have the ability to process
    // packets from external chips connected over serial.
    if constexpr (CPUSpeed == HalfClockSpeed) {
        ExecutionBody();
    } else if constexpr (CPUSpeed == QuarterClockSpeed) {
        ExecutionBody_5MHz();
    } 
}

extern "C" 
void displayAddress() noexcept {
    Serial.printf(F("0x%x'%x'%x'%x\n"),
            getInputRegister<AddressLines[3]>(),
            getInputRegister<AddressLines[2]>(),
            getInputRegister<AddressLines[1]>(),
            getInputRegister<AddressLines[0]>());
}
extern "C"
void displayReadTransaction() noexcept {
    Serial.println(F("Read Transaction!"));
}

extern "C"
void displaySignalReady() noexcept {
    Serial.println(F("Signal Ready and Waiting!"));
}

extern "C"
void displayDataPortValue() noexcept {
    Serial.printf(F("0x%x%x\n"), 
            getOutputRegister<DataLines[1]>(),
            getOutputRegister<DataLines[0]>());
}

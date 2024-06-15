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

#ifndef SXCHIPSET_TYPE103_PINOUT_H
#define SXCHIPSET_TYPE103_PINOUT_H
#include <Arduino.h>
/**
 * @brief Wrapper around the AVR Pins to make templating easier and cleaner
 */
enum class Pin : byte {
#define X(port, index) Port ## port ## index = PIN_P ## port ## index,
#include "AVRPins.def"
#ifdef NUM_DIGITAL_PINS
Count = NUM_DIGITAL_PINS,
#else
#error "Count must equal the number of digital pins available"
#endif
#undef X
#ifdef PIN_WIRE_SDA
    SDA = PIN_WIRE_SDA,
#endif
#ifdef PIN_WIRE_SCL
    SCL = PIN_WIRE_SCL,
#endif
#ifdef PIN_SPI_SS
    SPI_CS = PIN_SPI_SS,
#endif
#ifdef PIN_SPI_SCK
    SPI_SCK = PIN_SPI_SCK,
#endif
#ifdef PIN_SPI_MOSI
    SPI_MOSI = PIN_SPI_MOSI,
#endif
#ifdef PIN_SPI_MISO
    SPI_MISO = PIN_SPI_MISO,
#endif
#ifdef LED_BUILTIN
    LED = LED_BUILTIN,
#endif

    SD_EN = PortB0,
    XINT2 = PortB4,
    XINT0 = PortB5, // OC1A
    Lock = PortB6,
    Hold = PortB7,

    ADS = PortD2,
    BLAST = PortD3,
    WR = PortD4,
    BE0 = PortD5,
    BE1 = PortD6,
    IsIOOperation = PortD7,

    RXD0 = PortE0,
    TXD0 = PortE1,
    XINT6 = PortE2,
    XINT4 = PortE3,
    HLDA = PortE4,


    // PortE7 is free from emitting the 20MHz clock, this is handled by a
    // separate clock controller chip
    EBI_WR = PortG0,
    EBI_RD = PortG1,
    EBI_ALE = PortG2,
    Reset = PortG3,
    ReadyDirect = PortG4,

};
enum class Port : byte {
    // stop at mega2560 tier
#define X(name) name ,
#include "AVRPorts.def"
#undef X
    None,
    DataLinesLower = F,
    DataLinesUpper = C,
    AddressLinesLowest = K,
    AddressLinesLower = H,
    AddressLinesHigher = J,
    AddressLinesHighest = L,
};
constexpr Port AddressLines[4] {
    Port::AddressLinesLowest,
    Port::AddressLinesLower,
    Port::AddressLinesHigher,
    Port::AddressLinesHighest,
};
constexpr Port DataLines[2] {
    Port::DataLinesLower,
    Port::DataLinesUpper,
};
constexpr auto numberOfAvailablePins() noexcept {
    return 0 
#define X(a, b) + 1
#include "AVRPins.def"
#undef X
        ;
}
static_assert(numberOfAvailablePins() == NUM_DIGITAL_PINS);

constexpr auto numberOfAvailablePorts() noexcept {
    return 0
#define X(a) + 1
#include "AVRPorts.def"
#undef X
        ;
}

static_assert(numberOfAvailablePorts() > 0);

using PortOutputRegister = volatile byte&;
using PortInputRegister = volatile byte&;
using PortDirectionRegister = volatile byte&;
using PinState = decltype(LOW);
using PinDirection = decltype(OUTPUT);
struct FakeGPIOPort {
    uint8_t direction;
    uint8_t output;
    uint8_t input;
};
FakeGPIOPort& getFakePort() noexcept {
    static FakeGPIOPort port;
    return port;
}
constexpr Port getPort(Pin pin) noexcept {
    switch (pin) {
#define X(port, ind) case Pin :: Port ## port ## ind : return Port:: port ;
#include "AVRPins.def"
#undef X
        default: return Port::None;
    }
}
constexpr bool validPort(Port port) noexcept {
    switch (port) {
#define X(name) case Port :: name :
#include "AVRPorts.def"
#undef X
            return true;
        default:
            return false;
    }
}
[[gnu::always_inline]]
[[nodiscard]] constexpr decltype(auto) getPinMask(Pin pin) noexcept {
    switch (pin) {
#define X(port, offset) case Pin :: Port ## port ## offset : return _BV ( P ## port ## offset) ;
#include "AVRPins.def"
#undef X
        default:
            return 0xFF;
    }
}
template<Pin pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    return getPinMask(pin);
}
constexpr bool isPhysicalPin(Pin pin) noexcept {
    return validPort(getPort(pin));
}

template<Pin pin>
constexpr auto IsPhysicalPin_v = isPhysicalPin(pin);
[[gnu::always_inline]]
[[nodiscard]] 
inline PortOutputRegister 
getOutputRegister(Port port) noexcept {
    switch (port) {
#define X(name) case Port :: name : return PORT ## name ;
#include "AVRPorts.def"
#undef X
        default: 
            return getFakePort().output;
    }
}

template<Port port>
[[gnu::always_inline]]
[[nodiscard]] 
inline PortOutputRegister 
getOutputRegister() noexcept {
    return getOutputRegister(port);
}
template<Pin pin>
[[gnu::always_inline]]
[[nodiscard]] 
inline PortOutputRegister 
getOutputRegister() noexcept {
    return getOutputRegister<getPort(pin)>();
}

[[gnu::always_inline]]
[[nodiscard]] 
inline PortOutputRegister 
getOutputRegister(Pin pin) noexcept {
    return getOutputRegister(getPort(pin));
}
[[gnu::always_inline]]
[[nodiscard]] 
inline PortInputRegister 
getInputRegister(Port port) noexcept {
    switch (port) {
#define X(name) case Port :: name : return PIN ## name ;
#include "AVRPorts.def"
#undef X
        default: 
            return getFakePort().input;
    }
}
template<Port port>
[[gnu::always_inline]]
[[nodiscard]] 
inline PortInputRegister 
getInputRegister() noexcept {
    return getInputRegister(port);
}
template<Pin pin>
[[gnu::always_inline]]
[[nodiscard]] 
inline PortInputRegister 
getInputRegister() noexcept {
    return getInputRegister<getPort(pin)>();
}
[[gnu::always_inline]]
[[nodiscard]] 
inline PortInputRegister 
getInputRegister(Pin pin) noexcept {
    return getInputRegister(getPort(pin));
}
[[gnu::always_inline]]
[[nodiscard]] 
inline PortDirectionRegister 
getDirectionRegister(Port port) noexcept {
    switch (port) {
#define X(name) case Port:: name : return DDR ## name ;
#include "AVRPorts.def"
#undef X
        default: 
            return getFakePort().direction;
    }
}
template<Port port>
[[gnu::always_inline]]
[[nodiscard]] 
inline PortDirectionRegister 
getDirectionRegister() noexcept {
    return getDirectionRegister(port);
}
[[gnu::always_inline]]
//[[gnu::noinline]]
inline void
digitalWrite(Pin pin, PinState value) noexcept { 
    if (isPhysicalPin(pin)) {
        if (auto &thePort = getOutputRegister(pin); value == LOW) {
            thePort = thePort & ~getPinMask(pin);
        } else {
            thePort = thePort | getPinMask(pin);
        }
    } else {
        ::digitalWrite(static_cast<byte>(pin), value); 
    }
}

[[gnu::always_inline]] 
inline PinState 
digitalRead(Pin pin) noexcept { 
    if (isPhysicalPin(pin)) {
        return (getInputRegister(pin) & getPinMask(pin)) ? HIGH : LOW;
    } else {
        return ::digitalRead(static_cast<byte>(pin));
    }
}
template<Pin pin>
[[gnu::always_inline]] 
inline PinState
digitalRead() noexcept { 
    if constexpr (isPhysicalPin(pin)) {
        return (getInputRegister<pin>() & getPinMask<pin>()) ? HIGH : LOW;
    } else {
        return ::digitalRead(pin);
    }
}
[[gnu::always_inline]] inline 
void pinMode(Pin pin, PinDirection direction) noexcept {
    if (isPhysicalPin(pin)) {
        pinMode(static_cast<int>(pin), direction);
    } 
    // if the pin is not a physical one then don't expand it
}

template<Pin pin>
[[gnu::always_inline]] inline 
void pinMode(PinDirection direction) noexcept {
    if constexpr (isPhysicalPin(pin)) {
        pinMode(static_cast<int>(pin), direction);
    } 
    // if the pin is not a physical one then don't expand it
}

template<Pin pin>
[[gnu::always_inline]]
inline void
digitalWrite(PinState value) noexcept {
    if constexpr (isPhysicalPin(pin)) {
        if (auto& thePort = getOutputRegister<pin>(); value == LOW) {
            thePort = thePort & ~getPinMask<pin>();
        } else {
            thePort = thePort | getPinMask<pin>();
        }
    } else {
        digitalWrite(pin, value);
    }
}
template<Pin pin, PinState value>
[[gnu::always_inline]]
inline void
digitalWrite() noexcept {
    if constexpr (isPhysicalPin(pin)) {
        if constexpr (auto& thePort = getOutputRegister<pin>(); value == LOW) {
            thePort = thePort & ~getPinMask<pin>();
        } else {
            thePort = thePort | getPinMask<pin>();
        }
    } else {
        digitalWrite<pin>(value);
    }
}
template<Pin pin, PinState to, PinState from>
[[gnu::always_inline]]
inline void
pulse() noexcept {
    digitalWrite<pin, to>();
    digitalWrite<pin, from>();
}

/**
 * @brief Toggle the value of the output pin blindly
 * @tparam pin The output pin to toggle
 */
template<Pin pin>
[[gnu::always_inline]]
inline void
toggle() noexcept {
    getInputRegister<pin>() |= getPinMask<pin>();
}

/**
 * @brief Do a blind toggle of the given output pin by using the input register; This will always return the pin to its original position without needing to be explicit
 * @tparam pin The pin to toggle
 */
template<Pin pin>
[[gnu::always_inline]]
inline void
pulse() noexcept {
    toggle<pin>();
    toggle<pin>();
}

#endif // end SXCHIPSET_TYPE103_PINOUT_H

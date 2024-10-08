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

#ifndef CHIPSET2560_PINOUT_H
#define CHIPSET2560_PINOUT_H
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

#ifdef PIN_A0
    Analog0 = PIN_A0,
#endif
#ifdef PIN_A1
    Analog1 = PIN_A1,
#endif
#ifdef PIN_A2
    Analog2 = PIN_A2,
#endif
#ifdef PIN_A3
    Analog3 = PIN_A3,
#endif
#ifdef PIN_A4
    Analog4 = PIN_A4,
#endif
#ifdef PIN_A5
    Analog5 = PIN_A5,
#endif
#ifdef PIN_A6
    Analog6 = PIN_A6,
#endif
#ifdef PIN_A7
    Analog7 = PIN_A7,
#endif
#ifdef PIN_A8
    Analog8 = PIN_A8,
#endif
#ifdef PIN_A9
    Analog9 = PIN_A9,
#endif
#ifdef PIN_A10
    Analog10 = PIN_A10,
#endif
#ifdef PIN_A11
    Analog11 = PIN_A11,
#endif
#if NUM_ANALOG_INPUTS > 12
#ifdef PIN_A12
    Analog12 = PIN_A12,
#endif
#endif  // 13 entries
#if NUM_ANALOG_INPUTS > 13
#ifdef PIN_A13
    Analog13 = PIN_A13,
#endif
#endif // 14 entries
#if NUM_ANALOG_INPUTS > 14
#ifdef PIN_A14
    Analog14 = PIN_A14,
#endif 
#endif // 15 entries
#if NUM_ANALOG_INPUTS > 15
#ifdef PIN_A15
    Analog15 = PIN_A15,
#endif // A15
#endif // 16 inputs

};
enum class Port : byte {
    // stop at mega2560 tier
#define X(name) name ,
#include "AVRPorts.def"
#undef X
    None,
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
constexpr bool isAnalogPin(Pin p) noexcept {
    switch (p) {
#define X(idx) case Pin:: Analog ## idx : return true
#ifdef PIN_A0
        X(0);
#endif
#ifdef PIN_A1
        X(1);
#endif
#ifdef PIN_A2
        X(2);
#endif
#ifdef PIN_A3
        X(3);
#endif
#ifdef PIN_A4
        X(4);
#endif
#ifdef PIN_A5
        X(5);
#endif
#ifdef PIN_A6
        X(6);
#endif
#ifdef PIN_A7
        X(7);
#endif
#ifdef PIN_A8
        X(8);
#endif
#ifdef PIN_A9
        X(9);
#endif
#ifdef PIN_A10
        X(10);
#endif
#ifdef PIN_A11
        X(11);
#endif
#if NUM_ANALOG_INPUTS > 12
#ifdef PIN_A12
        X(12);
#endif
#endif
#if NUM_ANALOG_INPUTS > 13
#ifdef PIN_A13
        X(13);
#endif
#endif
#if NUM_ANALOG_INPUTS > 14
#ifdef PIN_A14
        X(14);
#endif
#endif
#if NUM_ANALOG_INPUTS > 15
#ifdef PIN_A15
        X(15);
#endif
#endif
#undef X

        default:
            return false;
    }
}

constexpr bool hasPWM(Pin p) noexcept {
    return digitalPinHasPWM(static_cast<byte>(p));
}

template<Pin pin>
constexpr auto hasPWM_v = hasPWM(pin);

template<Pin pin>
constexpr auto isAnalogPin_v = isAnalogPin(pin);

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
template<Port port>
constexpr bool validPort_v = validPort(port);

[[gnu::always_inline]]
[[nodiscard]] constexpr decltype(auto) getPinMask(Pin pin) noexcept {
    switch (pin) {
#ifdef MEGACOREX
#define X(port, offset) case Pin :: Port ## port ## offset : return PIN ## offset ## _bm ; 
#else
#define X(port, offset) case Pin :: Port ## port ## offset : return _BV ( P ## port ## offset) ;
#endif
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
#ifdef MEGACOREX
#define X(name) case Port :: name : return VPORT ## name . OUT;
#else
#define X(name) case Port :: name : return PORT ## name ;
#endif
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
#ifdef MEGACOREX
#define X(name) case Port :: name : return VPORT ## name . IN ;
#else
#define X(name) case Port :: name : return PIN ## name ;
#endif
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
#ifdef MEGACOREX
#define X(name) case Port:: name : return VPORT ## name . DIR ;
#else
#define X(name) case Port:: name : return DDR ## name ;
#endif
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
    static_assert(isPhysicalPin(pin));
    return (getInputRegister<pin>() & getPinMask<pin>()) ? HIGH : LOW;
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
    static_assert(isPhysicalPin(pin));
    pinMode(static_cast<int>(pin), direction);
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

[[gnu::always_inline]]
inline decltype(auto) analogRead(Pin pin) noexcept {
    return ::analogRead(static_cast<byte>(pin));
}
template<Pin pin>
[[gnu::always_inline]]
inline decltype(auto) analogRead() noexcept {
    static_assert(isAnalogPin_v<pin>, "Provided pin is not an analog pin!");
    return analogRead(pin);
}

#ifdef MEGACOREX
[[gnu::always_inline]]
inline decltype(auto) 
digitalReadFast(Pin pin) noexcept {
    return ::digitalReadFast(static_cast<byte>(pin));
}
[[gnu::always_inline]]
inline void 
digitalWriteFast(Pin pin, PinState state) noexcept {
    ::digitalWriteFast(static_cast<byte>(pin), state);
}
#endif  // end defined MEGACOREX

#endif // end CHIPSET2560_PINOUT_H

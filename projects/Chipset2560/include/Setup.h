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

#ifndef SXCHIPSET_TYPE103_SETUP_H__
#define SXCHIPSET_TYPE103_SETUP_H__
#include <Arduino.h>
#include "Detect.h"
#include "Types.h"
#include "Pinout.h"


template<uint8_t count>
[[gnu::always_inline]]
inline void
insertCustomNopCount() noexcept {
    __builtin_avr_nops(count);
}
[[gnu::always_inline]]
inline void
singleCycleDelay() noexcept {
    insertCustomNopCount<2>();
}

/**
 * @brief The CPU Module kind as described in the Cyclone SDK manual
 */
enum class CPUKind : uint8_t  {
    /// @brief The CPU installed is a 80960Sx processor
    Sx = 0b000,
    /// @brief The CPU installed is a 80960Kx processor
    Kx = 0b001,
    /// @brief The CPU installed is a 80960Cx processor
    Cx = 0b010,
    /// @brief The CPU installed is a 80960Hx processor
    Hx = 0b011,
    /// @brief The CPU installed is a 80960Jx processor
    Jx = 0b100,
    Reserved0 = 0b101,
    Reserved1 = 0b110,
    Reserved2 = 0b111,
    Generic,
};

enum class NativeBusWidth : uint8_t {
    Unknown,
    Sixteen,
    ThirtyTwo,
};

constexpr NativeBusWidth getBusWidth(CPUKind kind) noexcept {
    switch (kind) {
        case CPUKind::Sx:
            return NativeBusWidth::Sixteen;
        case CPUKind::Kx:
        case CPUKind::Cx:
        case CPUKind::Hx:
        case CPUKind::Jx:
            return NativeBusWidth::ThirtyTwo;
        default:
            return NativeBusWidth::Unknown;
    }
}


/// @brief The Cyclone SDK board supported many different clock speeds, we only support a 10MHz synchronized with the AVR chip
enum class CPUFrequencyInfo {
    /// @brief The only supported frequency
    Clock10MHz = 0b000,
};


constexpr bool alignedTo32bitBoundaries(uint32_t address) noexcept {
    return (address & 0b11) == 0;
}
constexpr bool isUpper16bitValue(uint32_t address) noexcept {
    return (address & 0b11) == 0b10;
}
constexpr bool isLower16bitValue(uint32_t address) noexcept {
    return alignedTo32bitBoundaries(address);
}
static_assert(alignedTo32bitBoundaries(0), "32-bit alignment logic is broken");
static_assert(!alignedTo32bitBoundaries(0b10), "32-bit alignment logic is broken");
static_assert(isUpper16bitValue(0b10), "is 16-bit upper half check is broken");
static_assert(!isUpper16bitValue(0), "is 16-bit upper half check is broken");
static_assert(isLower16bitValue(0), "is 16-bit lower half check is broken");
static_assert(!isLower16bitValue(0b10), "is 16-bit lower half check is broken");

#endif // end SXCHIPSET_TYPE103_SETUP_H__

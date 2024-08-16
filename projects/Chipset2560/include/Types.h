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

#ifndef CHIPSET2560_TYPES_H__
#define CHIPSET2560_TYPES_H__
#include <Arduino.h>

using Address = uint32_t;

template<typename W, typename E>
constexpr auto ElementCount = sizeof(W) / sizeof(E);
template<typename W, typename T>
using ElementContainer = T[ElementCount<W, T>];
template<typename T>
struct TypeTagDispatcher {
    using UnderlyingType = T;
};

template<auto value>
struct ValueTagDispatcher {
    static constexpr auto UnderlyingValue = value;
    using UnderlyingType = decltype(value);
    using TagDispatchType = TypeTagDispatcher<UnderlyingType>;

    static constexpr bool matches(UnderlyingType other) noexcept {
        return other == UnderlyingValue;
    }
};

template<typename T>
using TreatAs = TypeTagDispatcher<T>;
using TreatAsOrdinal = TreatAs<uint32_t>;

template<auto value>
using TagDispatchOnValue = ValueTagDispatcher<value>;

template<typename T>
using TagDispatchOnType = TreatAs<T>;

#endif //CHIPSET2560_TYPES_H__

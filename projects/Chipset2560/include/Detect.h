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

#ifndef SXCHIPSET_TYPE103_DETECT_H__
#define SXCHIPSET_TYPE103_DETECT_H__
#include <stdint.h>
#ifdef __AVR__
#include <avr/io.h>
#endif
#if defined(SPDR) && defined(SPIF) && defined(SPSR)
#define AVR_SPI_AVAILABLE
#endif

#ifndef GPIOR0
extern uint8_t GPIOR0;
#endif
#ifndef GPIOR1
extern uint8_t GPIOR1;
#endif
#ifndef GPIOR2
extern uint8_t GPIOR2;
#endif

constexpr auto getRAMEnd() noexcept {
    return RAMEND;
}
template<typename T>
inline volatile T* memoryPointer(size_t address) noexcept {
    return reinterpret_cast<volatile T*>(address) ;
}
template<typename T>
inline volatile T& memory(size_t address) noexcept {
    return *memoryPointer<T>(address);
}

template<typename T>
inline volatile T& adjustedMemory(size_t address) noexcept {
    return memory<T>(address + (getRAMEnd() + 1));
}

template<typename T>
inline volatile T* adjustedMemoryPointer(size_t address) noexcept {
    return memoryPointer<T>(address + (getRAMEnd() + 1));
}

#endif // end SXCHIPSET_TYPE103_DETECT_H__

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

#ifndef DECEPTION_H__
#define DECEPTION_H__
#include <Arduino.h>
namespace Deception {
    namespace MemoryCodes {
        constexpr uint8_t ReadMemoryCode = 0xFC;
        constexpr uint8_t WriteMemoryCode = 0xFD;
        constexpr uint8_t InitializeSystemSetupCode = 0xFE;
        constexpr uint8_t BeginInstructionCode = 0xFF;
    } // end namespace MemoryCodes
    using OnConnectionEstablishedCallback = bool(*)(HardwareSerial&);
    /**
     * @brief establish a connection on a hardware serial connection; will wait until a response
     * @param connection The hardware serial device to establish the connection across
     * @param callback The function to call after the connection has been established
     * @param waitBetween The number of milliseconds to wait between connection attempts (default 300)
     * @return the result from invoking the callback
     */
    bool establishContact(HardwareSerial& connection, OnConnectionEstablishedCallback callback, int waitBetween = 300) noexcept;
} // end namespace Deception
#endif // end DECEPTION_H__

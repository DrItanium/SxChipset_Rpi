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
#ifndef CHIPSET2560_PINS_H__
#define CHIPSET2560_PINS_H__
#include "Types.h"
#include "Pinout.h"
#include "Setup.h"

namespace Pins {
    constexpr auto SD_EN = Pin::PortB0;
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
    //constexpr auto PCJoystickInterrupt = Pin::PortE6;
    constexpr auto READY_SYNC_IN = Pin::PortE7;

    constexpr auto PSRAM_EN = Pin::PortG4;
    constexpr auto READY = Pin::PortG5;

} // end namepace Pins

namespace Ports {
    constexpr auto DataLower = Port::C;
    constexpr auto DataUpper = Port::F;
    constexpr auto PSRAMSel = Port::L;
} // end namespace Ports
#endif

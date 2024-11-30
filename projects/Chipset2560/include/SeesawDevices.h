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
#ifndef CHIPSET2560_SEESAW_DEVICES_H__
#define CHIPSET2560_SEESAW_DEVICES_H__
#include <Arduino.h>
#include <Adafruit_seesaw.h>

#include "Types.h"
#include "Setup.h"
#include "Pins.h"
#include "OptionalDevice.h"


namespace PCJoystick {
    constexpr auto Address = 0x49;
    constexpr auto Button1 = 3;
    constexpr auto Button2 = 13;
    constexpr auto Button3 = 2;
    constexpr auto Button4 = 14;
    constexpr uint32_t ButtonMask = (1ul << Button1) | (1ul << Button2) | (1ul << Button3) | (1ul << Button4);
    constexpr auto Joy1_X = 1;
    constexpr auto Joy1_Y = 15;
    constexpr auto Joy2_X = 0;
    constexpr auto Joy2_Y = 16;
    constexpr auto EnableInterrupt = false;
    constexpr uint32_t Version = 5753;
}
namespace GamepadQt {
    constexpr auto Address = 0x50;
    constexpr auto ButtonX = 6;
    constexpr auto ButtonY = 2;
    constexpr auto ButtonA = 5;
    constexpr auto ButtonB = 1;
    constexpr auto ButtonSelect = 0;
    constexpr auto ButtonStart = 16;
    constexpr uint32_t ButtonMask = (1ul << ButtonX) | (1ul << ButtonY) | (1ul << ButtonA) | (1ul << ButtonB) | (1ul << ButtonSelect) | (1ul << ButtonStart);
    constexpr auto EnableInterrupt = false;
    constexpr auto Joy_X = 14;
    constexpr auto Joy_Y = 15;
    constexpr uint32_t Version = 5743;
}

using SeesawDevice = OptionalDevice<Adafruit_seesaw>;
#endif

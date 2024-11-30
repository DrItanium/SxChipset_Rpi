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
#ifndef CHIPSET2560_OPTIONALDEVICE_H__
#define CHIPSET2560_OPTIONALDEVICE_H__
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
        void disable() noexcept { _valid = false; }
    private:
        bool _valid = false;
        T _device;
};
#endif

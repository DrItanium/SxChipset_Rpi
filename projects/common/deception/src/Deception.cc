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

#include <Deception.h>

namespace Deception {

bool
establishContact(HardwareSerial& connection, OnConnectionEstablishedCallback callback, int waitBetween) noexcept {
    while (connection.available() <= 0) {
        connection.write(MemoryCodes::InitializeSystemSetupCode);
        delay(waitBetween);
    }
    // invoke the callback
    if (callback) {
        return callback(connection);
    } else {
        return true;
    }
}

void
CacheLine::clear() noexcept {
    _dirty = false;
    _backingStore = nullptr;
    _key = 0;
    for (auto i = 0u; i < NumBytes; ++i) {
        _bytes[i] = 0;
    }
}
void
CacheLine::sync() noexcept {
    if (valid()) {
        if (_dirty) {
            _dirty = false;
            (void)_backingStore->write(_key, _bytes, NumBytes);
        }
    }
}
void
CacheLine::replace(BackingStore& store, Address newAddress) noexcept {
    sync();
    _backingStore = &store;
    _key = normalizeAddress(newAddress);
    (void)_backingStore->read(_key, _bytes, NumBytes);
}

void
CacheLine::setByte(uint8_t offset, uint8_t value) noexcept {
    _dirty = true;
    _bytes[computeByteOffset(offset)] = value;
}

} // end namespace Deception

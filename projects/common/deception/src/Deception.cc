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


MemoryCodes
TwoWireBackingStore::backingStoreStatus(uint8_t size) noexcept {
    _link.requestFrom(_index, size);
    return static_cast<MemoryCodes>(_link.read());
}

bool
TwoWireBackingStore::backingStoreBooting() noexcept {
    return backingStoreStatus(17) == MemoryCodes::BootingUp;
}
void
TwoWireBackingStore::waitForBackingStoreIdle() noexcept {
    while (true) {
        switch (backingStoreStatus(17)) {
            case MemoryCodes::CurrentlyProcessingRequest:
            case MemoryCodes::RequestedData:
            case MemoryCodes::NothingToProvide:
                return;
            default:
                break;
        }
    }
}
void
TwoWireBackingStore::waitForMemoryReadSuccess(uint8_t amount) noexcept {
    while (true) {
        switch (backingStoreStatus(amount)) {
            case MemoryCodes::RequestedData:
                return;
            default:
                break;
        }
    }
}

size_t 
TwoWireBackingStore::write(Address addr, uint8_t* storage, size_t count) noexcept {
    waitForBackingStoreIdle();
    _link.beginTransmission(_index);
    _link.write(static_cast<uint8_t>(Deception::MemoryCodes::WriteMemory));
    _link.write(reinterpret_cast<uint8_t*>(&addr), sizeof(Address));
    _link.write(static_cast<uint8_t>(count));
    _link.write(storage, count);
    _link.endTransmission();
    return count;
}

size_t 
TwoWireBackingStore::read(Address addr, uint8_t* storage, size_t count) noexcept {
    waitForBackingStoreIdle();
    _link.beginTransmission(_index);
    _link.write(static_cast<uint8_t>(Deception::MemoryCodes::ReadMemory));
    _link.write(reinterpret_cast<uint8_t*>(&addr), sizeof(Address));
    // Just send the lowest 8 bits of data, this could case a strange mismatch
    _link.write(static_cast<uint8_t>(count));
    _link.endTransmission();
    waitForMemoryReadSuccess(static_cast<uint8_t>(count) + 1); // add one for the return code
    size_t i = 0;
    while (1 < Wire.available() ) {
        storage[i] = Wire.read();
        ++i;
    }
    storage[i] = Wire.read();
    ++i;
    return i;
}

} // end namespace Deception

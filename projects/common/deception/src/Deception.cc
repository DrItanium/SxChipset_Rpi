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


void
CacheLine16::clear() noexcept {
    _dirty = false;
    _backingStore = nullptr;
    _key = 0;
    for (auto i = 0u; i < NumBytes; ++i) {
        _bytes[i] = 0;
    }
}
void
CacheLine16::sync() noexcept {
    if (valid()) {
        if (_dirty) {
            _dirty = false;
            (void)_backingStore->write(_key, _bytes, NumBytes);
        }
    }
}
void
CacheLine16::replace(BackingStore& store, Address newAddress) noexcept {
    sync();
    _backingStore = &store;
    _key = normalizeAddress(newAddress);
    (void)_backingStore->read(_key, _bytes, NumBytes);
}

void
CacheLine16::setByte(uint8_t offset, uint8_t value) noexcept {
    markDirty();
    _bytes[computeByteOffset(offset)] = value;
}

void
CacheLine16::markDirty() noexcept {
    _dirty = true;
}
void
CacheLine32::clear() noexcept {
    _dirty = false;
    _backingStore = nullptr;
    _key = 0;
    for (auto i = 0u; i < NumBytes; ++i) {
        _bytes[i] = 0;
    }
}
void
CacheLine32::sync() noexcept {
    if (valid()) {
        if (_dirty) {
            _dirty = false;
            (void)_backingStore->write(_key, _bytes, NumBytes);
        }
    }
}
void
CacheLine32::replace(BackingStore& store, Address newAddress) noexcept {
    sync();
    _backingStore = &store;
    _key = normalizeAddress(newAddress);
    (void)_backingStore->read(_key, _bytes, NumBytes);
}

void
CacheLine32::setByte(uint8_t offset, uint8_t value) noexcept {
    markDirty();
    _bytes[computeByteOffset(offset)] = value;
}

void
CacheLine64::clear() noexcept {
    _dirty = false;
    _backingStore = nullptr;
    _key = 0;
    for (auto i = 0u; i < NumBytes; ++i) {
        _bytes[i] = 0;
    }
}
void
CacheLine64::sync() noexcept {
    if (valid()) {
        if (_dirty) {
            _dirty = false;
            (void)_backingStore->write(_key, _bytes, NumBytes);
        }
    }
}
void
CacheLine64::replace(BackingStore& store, Address newAddress) noexcept {
    sync();
    _backingStore = &store;
    _key = normalizeAddress(newAddress);
    (void)_backingStore->read(_key, _bytes, NumBytes);
}

void
CacheLine64::setByte(uint8_t offset, uint8_t value) noexcept {
    markDirty();
    _bytes[computeByteOffset(offset)] = value;
}
template<typename T>
inline
void
sendCommandHeader(T& link, uint8_t size, uint8_t code) noexcept {
    link.write(MemoryCodes::BeginInstructionCode);
    link.write(size);
    link.write(code);
}
template<typename T>
inline
void
send32BitNumber(T& link, uint32_t number) {
    link.write(reinterpret_cast<char*>(&number), sizeof(number));
}

template<typename T>
void 
writeMemoryBlock(T& link, Address address, uint8_t* data, uint8_t size) noexcept {
    sendCommandHeader(link, 1 + sizeof(address) + size + 1, MemoryCodes::WriteMemoryCode);
    send32BitNumber(link, address);
    link.write(size);
    link.write(data, size);
}
template<typename T>
void 
readMemoryBlock(T& link, Address address, uint8_t* data, uint8_t size) noexcept {
    sendCommandHeader(link, 1 + sizeof(address) + 1, MemoryCodes::ReadMemoryCode);
    send32BitNumber(link, address);
    link.write(size);
    for (uint8_t i = 0; i < size; ) {
        if (auto result = link.read(); result != -1) {
            data[i] = result;
            ++i;
        }
    }
}

size_t 
StreamBackingStore::read(Address addr, uint8_t* storage, size_t count) noexcept {
    readMemoryBlock(_link, addr, storage, count);
    return count;
}
size_t 
StreamBackingStore::write(Address addr, uint8_t* storage, size_t count) noexcept {
    writeMemoryBlock(_link, addr, storage, count);
    return count;
}

uint8_t
TwoWireBackingStore::backingStoreStatus(uint8_t size) noexcept {
    _link.requestFrom(_index, size);
    return _link.read();
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
    _link.write(Deception::MemoryCodes::WriteMemoryCode);
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
    _link.write(Deception::MemoryCodes::ReadMemoryCode);
    _link.write(reinterpret_cast<uint8_t*>(&addr), sizeof(Address));
    // Just send the lowest 8 bits of data, this could case a strange mismatch
    _link.write(static_cast<uint8_t>(count));
    _link.endTransmission();
    waitForMemoryReadSuccess(static_cast<uint8_t>(count) + 1);
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

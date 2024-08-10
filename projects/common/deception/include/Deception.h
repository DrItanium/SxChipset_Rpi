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
    using Address = uint32_t;
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
    
    /**
     * @brief An abstract representation of backing storage (memory, disk, etc)
     */
    class BackingStore {
        public:
            virtual ~BackingStore() = default;
            virtual size_t read(Address targetAddress, uint8_t* storage, size_t count) noexcept = 0;
            virtual size_t write(Address targetAddress, uint8_t* storage, size_t count) noexcept = 0;
            virtual bool begin() noexcept = 0;
    };

    /**
     * @brief A backing store that is really just a sink that acts as a
     * fallback in the cases where we are not mapped to anything
     */
    class DummyBackingStore final : public BackingStore {
        public:
            ~DummyBackingStore() override = default;
            bool begin() noexcept override { return true; }
            size_t read(Address, uint8_t* storage, size_t count) noexcept override {
                // clear the memory
                for (size_t i = 0; i < count; ++i) {
                    storage[i] = 0;
                }
                return count;
            }
            size_t write(Address, uint8_t*, size_t count) noexcept override {
                // do nothing
                return count;
            }
    };
    constexpr bool isPowerOfTwo(uint16_t value) noexcept {
        switch (value) {
            case 0b0000000000000001:
            case 0b0000000000000010:
            case 0b0000000000000100:
            case 0b0000000000001000:
            case 0b0000000000010000:
            case 0b0000000000100000:
            case 0b0000000001000000:
            case 0b0000000010000000:
            case 0b0000000100000000:
            case 0b0000001000000000:
            case 0b0000010000000000:
            case 0b0000100000000000:
            case 0b0001000000000000:
            case 0b0010000000000000:
            case 0b0100000000000000:
            case 0b1000000000000000:
                return true;
            default:
                return false;
        }
    }
    /**
     * @brief A simple 16-byte cache line
     */
    class CacheLine {
        public:
            static constexpr uint8_t NumBytes = 16;
            static constexpr auto ShiftAmount = 4;
            static_assert(isPowerOfTwo(NumBytes), "Cache Line size must be a power of two");
            static constexpr Address AddressMask = 0xFFFFFFF0;
            static constexpr Address normalizeAddress(Address input) noexcept {
                return input & AddressMask;
            }
            static constexpr uint8_t computeByteOffset(uint8_t input) noexcept {
                return input & 0xF;
            }
            constexpr bool dirty() const noexcept { return _dirty; }
            constexpr bool valid() const noexcept { return _backingStore != nullptr; }
            constexpr bool matches(Address other) const noexcept {
                return valid() && (_key == normalizeAddress(other));
            }
            void sync() noexcept;
            void replace(BackingStore& store, Address newAddress) noexcept;
            void setByte(uint8_t offset, uint8_t value) noexcept;
            constexpr uint8_t getByte(uint8_t offset) const noexcept {
                return _bytes[computeByteOffset(offset)];
            }
            /**
             * @brief Clear the contents of the line without syncing or anything like that
             */
            void clear() noexcept;
        private:
            uint8_t _bytes[NumBytes] = { 0 };
            uint32_t _key = 0;
            bool _dirty = false;
            BackingStore* _backingStore = nullptr;
    };
    template<uint16_t C, typename L = CacheLine>
    class DirectMappedCache {
        public:
            static_assert(C > 0, "Must have at least one cache line!");
            static_assert(isPowerOfTwo(C), "The given line size is not a power of two!");
            static constexpr auto NumLines = C;
            static constexpr auto LineMask = NumLines - 1;
            using Line_t = L;
            static constexpr Address computeIndex(Address input) noexcept {
                return (input >> Line_t::ShiftAmount) & LineMask;
            }
            void clear() noexcept {
                for (auto& line : lines) {
                    line.clear();
                }
            }
            Line_t& find(BackingStore& store, Address address) noexcept {
                auto& line = lines[computeIndex(address)];
                if (!line.matches(address)) {
                    line.replace(store, address);
                }
                return line;
            }
            void begin() noexcept {
                clear();
            }
        private:
            Line_t lines[NumLines];
    };
    using DirectMappedCache4K = DirectMappedCache<256>;
    static_assert(DirectMappedCache4K::computeIndex(0xFFFF'FFFF) == 0xFF);
} // end namespace Deception
#endif // end DECEPTION_H__
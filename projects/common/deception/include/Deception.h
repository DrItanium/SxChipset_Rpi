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
#include <Wire.h>

namespace Deception {
    using Address = uint32_t;
    constexpr uint32_t PCLinkSpeed = 9600;
    constexpr uint8_t TWI_MemoryControllerIndex = 0x2f;
    constexpr uint32_t TWI_ClockRate = 400'000;
    enum class MemoryCodes : uint8_t {
        ReadMemory = 0xC0,
        WriteMemory,
        BeginInstruction,
        CurrentlyProcessingRequest,
        BootingUp,
        RequestedData,
        NothingToProvide,
        SelectDevice,
        ConfigureCPUClockMode,
        SetMode = SelectDevice,

    };
    enum class SelectedDevice : uint8_t {
        CPUClockConfiguration,
        CPUClockConfiguration_CLK2,
        CPUClockConfiguration_CLK1,
        AnalogSensors,
        AnalogSensors_Channel0,
        AnalogSensors_Channel1,
        AnalogSensors_Channel2,
        AnalogSensors_Channel3,
        AnalogSensors_Channel4,
        AnalogSensors_Channel5,
        AnalogSensors_Channel6,
        AnalogSensors_Channel7,
        AnalogSensors_Channel8,
        AnalogSensors_Channel9,
        AnalogSensors_Channel10,
        TimeSinceStartup,
    };
    /**
     * @brief An abstract representation of backing storage (memory, disk, etc)
     */
    class BackingStore {
        public:
            virtual ~BackingStore() = default;
            virtual size_t read(Address targetAddress, uint8_t* storage, size_t count) noexcept = 0;
            virtual size_t write(Address targetAddress, uint8_t* storage, size_t count) noexcept = 0;
    };
    /**
     * @brief A backing store that is really just a sink that acts as a
     * fallback in the cases where we are not mapped to anything
     */
    class DummyBackingStore final : public BackingStore {
        public:
            ~DummyBackingStore() override = default;
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

    class TwoWireBackingStore : public BackingStore {
        public:
            TwoWireBackingStore(TwoWire& link, uint8_t index) : _link(link), _index(index) { }
            ~TwoWireBackingStore() override = default;
            size_t read(Address addr, uint8_t* storage, size_t count) noexcept override;
            size_t write(Address addr, uint8_t* storage, size_t count) noexcept override;
            void waitForBackingStoreIdle() noexcept;
        private:
            [[nodiscard]] MemoryCodes backingStoreStatus(uint8_t size) noexcept;
            [[nodiscard]] bool backingStoreBooting() noexcept;
            void waitForMemoryReadSuccess(uint8_t size) noexcept;
        private:
            TwoWire& _link;
            uint8_t _index;
    };
    constexpr bool isPowerOfTwo(uint64_t value) noexcept {
        switch (value) {
            case 0b0000000000000000000000000000000000000000000000000000000000000001:
            case 0b0000000000000000000000000000000000000000000000000000000000000010:
            case 0b0000000000000000000000000000000000000000000000000000000000000100:
            case 0b0000000000000000000000000000000000000000000000000000000000001000:
            case 0b0000000000000000000000000000000000000000000000000000000000010000:
            case 0b0000000000000000000000000000000000000000000000000000000000100000:
            case 0b0000000000000000000000000000000000000000000000000000000001000000:
            case 0b0000000000000000000000000000000000000000000000000000000010000000:
            case 0b0000000000000000000000000000000000000000000000000000000100000000:
            case 0b0000000000000000000000000000000000000000000000000000001000000000:
            case 0b0000000000000000000000000000000000000000000000000000010000000000:
            case 0b0000000000000000000000000000000000000000000000000000100000000000:
            case 0b0000000000000000000000000000000000000000000000000001000000000000:
            case 0b0000000000000000000000000000000000000000000000000010000000000000:
            case 0b0000000000000000000000000000000000000000000000000100000000000000:
            case 0b0000000000000000000000000000000000000000000000001000000000000000:
            case 0b0000000000000000000000000000000000000000000000010000000000000000:
            case 0b0000000000000000000000000000000000000000000000100000000000000000:
            case 0b0000000000000000000000000000000000000000000001000000000000000000:
            case 0b0000000000000000000000000000000000000000000010000000000000000000:
            case 0b0000000000000000000000000000000000000000000100000000000000000000:
            case 0b0000000000000000000000000000000000000000001000000000000000000000:
            case 0b0000000000000000000000000000000000000000010000000000000000000000:
            case 0b0000000000000000000000000000000000000000100000000000000000000000:
            case 0b0000000000000000000000000000000000000001000000000000000000000000:
            case 0b0000000000000000000000000000000000000010000000000000000000000000:
            case 0b0000000000000000000000000000000000000100000000000000000000000000:
            case 0b0000000000000000000000000000000000001000000000000000000000000000:
            case 0b0000000000000000000000000000000000010000000000000000000000000000:
            case 0b0000000000000000000000000000000000100000000000000000000000000000:
            case 0b0000000000000000000000000000000001000000000000000000000000000000:
            case 0b0000000000000000000000000000000010000000000000000000000000000000:
            case 0b0000000000000000000000000000000100000000000000000000000000000000:
            case 0b0000000000000000000000000000001000000000000000000000000000000000:
            case 0b0000000000000000000000000000010000000000000000000000000000000000:
            case 0b0000000000000000000000000000100000000000000000000000000000000000:
            case 0b0000000000000000000000000001000000000000000000000000000000000000:
            case 0b0000000000000000000000000010000000000000000000000000000000000000:
            case 0b0000000000000000000000000100000000000000000000000000000000000000:
            case 0b0000000000000000000000001000000000000000000000000000000000000000:
            case 0b0000000000000000000000010000000000000000000000000000000000000000:
            case 0b0000000000000000000000100000000000000000000000000000000000000000:
            case 0b0000000000000000000001000000000000000000000000000000000000000000:
            case 0b0000000000000000000010000000000000000000000000000000000000000000:
            case 0b0000000000000000000100000000000000000000000000000000000000000000:
            case 0b0000000000000000001000000000000000000000000000000000000000000000:
            case 0b0000000000000000010000000000000000000000000000000000000000000000:
            case 0b0000000000000000100000000000000000000000000000000000000000000000:
            case 0b0000000000000001000000000000000000000000000000000000000000000000:
            case 0b0000000000000010000000000000000000000000000000000000000000000000:
            case 0b0000000000000100000000000000000000000000000000000000000000000000:
            case 0b0000000000001000000000000000000000000000000000000000000000000000:
            case 0b0000000000010000000000000000000000000000000000000000000000000000:
            case 0b0000000000100000000000000000000000000000000000000000000000000000:
            case 0b0000000001000000000000000000000000000000000000000000000000000000:
            case 0b0000000010000000000000000000000000000000000000000000000000000000:
            case 0b0000000100000000000000000000000000000000000000000000000000000000:
            case 0b0000001000000000000000000000000000000000000000000000000000000000:
            case 0b0000010000000000000000000000000000000000000000000000000000000000:
            case 0b0000100000000000000000000000000000000000000000000000000000000000:
            case 0b0001000000000000000000000000000000000000000000000000000000000000:
            case 0b0010000000000000000000000000000000000000000000000000000000000000:
            case 0b0100000000000000000000000000000000000000000000000000000000000000:
            case 0b1000000000000000000000000000000000000000000000000000000000000000:
                return true;
            default:
                return false;
        }
    }
    template<uint8_t size, uint8_t shift, Address mask, typename T = BackingStore>
    struct CacheLine {
        using BackingStore_t = T;
        static constexpr auto NumBytes = size;
        static constexpr auto ShiftAmount = shift;
        static constexpr auto AddressMask = mask;
        static constexpr auto ByteOffsetMask = static_cast<uint8_t>(~AddressMask);
        static_assert(isPowerOfTwo(NumBytes), "CacheLine size must be a power of two");
        static_assert(NumBytes <= 128, "Cache Line Size is too large!");
        static constexpr Address normalizeAddress(Address input) noexcept {
            return input & AddressMask;
        }
        static constexpr uint8_t computeByteOffset(uint8_t input) noexcept {
            return input & ByteOffsetMask;
        }
        constexpr bool dirty() const noexcept { return _dirty; }
        constexpr bool valid() const noexcept { return _backingStore != nullptr; }
        constexpr bool matches(Address other) const noexcept { return valid() && (_key == normalizeAddress(other)); }
        void replace(BackingStore_t& store, Address newAddress) noexcept {
            if (valid()) {
                if (_dirty) {
                    _dirty = false;
                    (void)_backingStore->write(_key, _bytes, NumBytes);
                }
            }
            _backingStore = &store;
            _key = normalizeAddress(newAddress);
            (void)_backingStore->read(_key, _bytes, NumBytes);
        }
        void setByte(uint8_t offset, uint8_t value) noexcept {
            markDirty();
            _bytes[computeByteOffset(offset)] = value;
        }
        constexpr uint8_t getByte(uint8_t offset) const noexcept {
            return _bytes[computeByteOffset(offset)];
        }
        void clear() noexcept {
            _dirty = false;
            _backingStore = nullptr;
            _key = 0;
            for (auto i = 0u; i < NumBytes; ++i) {
                _bytes[i] = 0;
            }
        }
        uint8_t* getLineData(uint8_t offset = 0) noexcept {
            return &_bytes[offset];
        }
        void markDirty() noexcept { _dirty = true; }
        private:
            uint8_t _bytes[NumBytes] = { 0 };
            uint32_t _key = 0;
            bool _dirty = false;
            BackingStore_t* _backingStore = nullptr;
    };
    template<typename T = BackingStore>
    using CacheLine16 = CacheLine<16, 4, 0xFFFF'FFF0, T>;
    template<typename T = BackingStore>
    using CacheLine32 = CacheLine<32, 5, 0xFFFF'FFE0, T>;
    template<typename T = BackingStore>
    using CacheLine64 = CacheLine<64, 6, 0xFFFF'FFC0, T>;
    template<typename T = BackingStore>
    using CacheLine128 = CacheLine<128, 7, 0xFFFF'FF80, T>;
    template<uint16_t C, typename L>
    class DirectMappedCache {
        public:
            static_assert(C > 0, "Must have at least one cache line!");
            static_assert(isPowerOfTwo(C), "The given line size is not a power of two!");
            static constexpr auto NumLines = C;
            static constexpr auto LineMask = NumLines - 1;
            using Line_t = L;
            using BackingStore_t = typename Line_t::BackingStore_t;
            static constexpr uint8_t computeIndex(Address input) noexcept {
                return (static_cast<uint16_t>(input) >> Line_t::ShiftAmount) & LineMask;
            }
            static constexpr uint8_t computeOffset(uint8_t input) noexcept {
                return Line_t::computeByteOffset(input);
            }
            void clear() noexcept {
                for (auto line : lines) {
                    line.clear();
                }
            }
            Line_t& find(BackingStore_t& store, Address address) noexcept {
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
    template<uint16_t C, typename L>
    class TwoWayCache {
        public:
            static_assert(C > 0, "Must have at least one cache line!");
            static_assert(isPowerOfTwo(C), "The given line size is not a power of two!");
            static constexpr auto NumLines = C;
            static constexpr auto NumSets = NumLines / 2;
            static constexpr auto LineMask = NumSets - 1;
            using Line_t = L;
            using BackingStore_t = typename Line_t::BackingStore_t;
            struct CacheSet {
                bool lastElement = true;
                Line_t lines[2];
                void clear() noexcept {
                    lastElement = true;
                    lines[0].clear();
                    lines[1].clear();
                }

                Line_t& find(BackingStore_t& store, Address address) noexcept {
                    if (lines[0].matches(address)) {
                        lastElement = false;
                        return lines[0];
                    } else if (lines[1].matches(address)) {
                        lastElement = true;
                        return lines[1];
                    } else {
                        // no match
                        auto index = lastElement ? 0 : 1;
                        lastElement = !lastElement;
                        lines[index].replace(store, address);
                        return lines[index];
                    }
                }
            };
            using Set_t = CacheSet;
            static constexpr uint8_t computeIndex(Address input) noexcept {
                return (static_cast<uint16_t>(input) >> Line_t::ShiftAmount) & LineMask;
            }
            static constexpr uint8_t computeOffset(uint8_t input) noexcept {
                return Line_t::computeByteOffset(input);
            }
            void clear() noexcept {
                for (auto set : sets) {
                    set.clear();
                }
            }
            Line_t& find(BackingStore_t& store, Address address) noexcept {
                return sets[computeIndex(address)].find(store, address);
            }
            void begin() noexcept {
                clear();
            }
        private:
            Set_t sets[NumSets];
    };

} // end namespace Deception
#endif // end DECEPTION_H__

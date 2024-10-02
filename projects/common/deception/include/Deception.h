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

    class TwoWireBackingStore {
        public:
            TwoWireBackingStore(TwoWire& link, uint8_t index) : _link(link), _index(index) { }
            size_t read(Address addr, uint8_t* storage, size_t count) noexcept;
            size_t write(Address addr, uint8_t* storage, size_t count) noexcept;
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
    template<typename A, uint8_t size, uint8_t shift, A mask, typename T>
    struct CacheLine {
        using BackingStore_t = T;
        using Address_t = A;
        static constexpr auto NumBytes = size;
        static constexpr auto ShiftAmount = shift;
        static constexpr auto AddressMask = mask;
        static constexpr auto ByteOffsetMask = static_cast<uint8_t>(~AddressMask);
        static_assert(isPowerOfTwo(NumBytes), "CacheLine size must be a power of two");
        static_assert(NumBytes <= 128, "Cache Line Size is too large!");
        static constexpr Address_t normalizeAddress(Address_t input) noexcept {
            return input & AddressMask;
        }
        static constexpr uint8_t computeByteOffset(uint8_t input) noexcept {
            return input & ByteOffsetMask;
        }
        constexpr bool dirty() const noexcept { return _dirty; }
        constexpr bool matches(Address_t other) const noexcept { return (_key == other); }
        void replace(BackingStore_t& store, Address_t newAddress) noexcept {
            if (_dirty) {
                (void)store.write(_key, _bytes, NumBytes);
            }
            load(store, newAddress);
        }
        void load(BackingStore_t& store, Address_t newAddress) noexcept {
            _dirty = false;
            _key = newAddress;
            (void)store.read(_key, _bytes, NumBytes);
        }
        void setByte(uint8_t offset, uint8_t value) noexcept {
            markDirty();
            _bytes[computeByteOffset(offset)] = value;
        }
        constexpr uint8_t getByte(uint8_t offset) const noexcept {
            return _bytes[computeByteOffset(offset)];
        }
        void clear() noexcept {
            _key = 0;
            _dirty = false;
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
            Address_t _key = 0;
            bool _dirty = false;
    };
    template<typename A, typename T>
    using CacheLine16 = CacheLine<A, 16, 4, static_cast<A>(0xFFFF'FFF0), T>;
    template<typename A, typename T>
    using CacheLine32 = CacheLine<A, 32, 5, static_cast<A>(0xFFFF'FFE0), T>;
    template<uint16_t C, typename L>
    class DirectMappedCache {
        public:
            static_assert(C > 0, "Must have at least one cache line!");
            static_assert(isPowerOfTwo(C), "The given line size is not a power of two!");
            static constexpr auto NumLines = C;
            static constexpr auto LineMask = NumLines - 1;
            using Line_t = L;
            using Address_t = typename Line_t::Address_t;
            using BackingStore_t = typename Line_t::BackingStore_t;
            static constexpr auto NumBytesPerLine = Line_t::NumBytes;
            static constexpr auto NumCacheBytes = NumLines * NumBytesPerLine;
            template<bool useOldIndexComputation = false>
            [[gnu::used]]
            static constexpr uint8_t computeIndex(Address_t input) noexcept {
                if constexpr (useOldIndexComputation) {
                    return (static_cast<uint16_t>(input) >> Line_t::ShiftAmount) & LineMask;
                } else {
                    // In order to eliminate the expensive right shifts that
                    // come with the old design, we treat the lower 16-bits of
                    // the address as two 8-bit numbers that are combined
                    // through either add or xor (right now it is add). 
                    //
                    // What is going on is that the index is now the lower byte
                    // and the lowest 4 bits of the key are now in the lowest
                    // byte.
                    //
                    // So if you have 0xFDEDABCD you first get 0xFDEDABC0 and
                    // then we take the lower half and use it to compute the
                    // index. So we would have 0xABC0. Since the original index
                    // 0xBC required multiple expensive shift operations to
                    // extract we need to do something different. So now we
                    // treat 0xAB as the index (the compiler does not need to
                    // do anything since it will already be stored in a
                    // collection of registers) with the lowest byte (0xC0)
                    // being the offset. At this point we will then just
                    // combine the two numbers together (which also requires no
                    // register movement). The result is the cache index. 
                    //
                    // This is _much_ faster overall because of its lack of
                    // register movement. It does mean that key compares have
                    // to compare the entire key (not a subset). The base must
                    // be included since you cannot eliminate it from the
                    // number (since we could reconstruct). 
                    //
                    // This isn't actually an issue since I have been storing
                    // the key as the full base address with no modification.
                    // This is not only much faster but also can be expanded to
                    // control how the comparison operation works. 
                    //
                    // This modification has the side effect of cache lines
                    // being used in a different order than previously. So,
                    // 0x0000 will translate to line 0 in the cache. 0x0010
                    // will translate to line 16 in the cache. And so on, the
                    // combination should also prevent some forms of thrashing
                    // that happen on 4k boundaries. 
                    //
                    // This is very important because one of the alignment
                    // directives in the i960 assembler is to align it to 4k
                    // boundaries. It also means that copying from addresses
                    // like 0x1000 to an address like 0x2000 will no longer
                    // collide. Instead, the lines will be stashed within the
                    // cache at different offsets (0x10 and 0x20 respectively). 
                    //
                    // Obviously, thrashing can still happen but the
                    // rearrangement of the lowest 4bits of the key and the
                    // index should eliminate some of the nastiest problems.
                    //
                    //
                    // This design is also just as robust with larger cache
                    // lines too. The masking is just a little different. We
                    // just have a different mask overall. 
                    //
                    // With a 32-byte cache line, the lowest 5-bits are ignored
                    // but we also have 128 cache lines. Thus we need to mask
                    // out the index at the end.
                    //
                    // We do not care if the key bits leak into the base index
                    // with this design since we are going to mask it down at
                    // the end. If you use a 256 entry cache then there is no
                    // extra step for masking but even with that being said. It
                    // is a very simple mask operation
                    uint8_t base = static_cast<uint8_t>(input >> 8);
                    // we assume that the masking of the line offset already took
                    // place.
                    uint8_t offset = static_cast<uint8_t>(input);
                    // we only perform the masking at the end to make sure that
                    // the index isn't out of bounds
                    return (base ^ offset) & LineMask;
                }
            }
            static constexpr uint8_t computeOffset(uint8_t input) noexcept {
                return Line_t::computeByteOffset(input);
            }
            static constexpr Address_t normalizeAddress(Address_t input) noexcept {
                return Line_t::normalizeAddress(input);
            }
            void clear() noexcept {
                for (auto line : lines) {
                    line.clear();
                }
            }
            Line_t& find(BackingStore_t& store, Address_t address) noexcept {
                auto addr = normalizeAddress(address);
                auto& line = lines[computeIndex(addr)];
                if (!line.matches(addr)) {
                    line.replace(store, addr);
                } 
                return line;
            }
            void seed(BackingStore_t& store, Address_t address) noexcept {
                auto addr = normalizeAddress(address);
                auto& line = lines[computeIndex(addr)];
                line.load(store, addr);
            }
            void begin() noexcept {
                clear();
            }
        private:
            Line_t lines[NumLines];
    };


} // end namespace Deception
#endif // end DECEPTION_H__

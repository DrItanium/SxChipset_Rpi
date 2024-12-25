#ifndef EXPERIMENT_PLATFORM_CONCEPTS_H__
#define EXPERIMENT_PLATFORM_CONCEPTS_H__

#include <Arduino.h>
#include <microshell.h>

#define FILE_DESCRIPTOR_ARGS struct ush_object* self, struct ush_file_descriptor const * file
#define PASS_FILE_DESCRIPTOR_ARGS self, file
#define ByteFile(n, reg) { \
        .name = n , \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) noexcept { \
            return sendByte(PASS_FILE_DESCRIPTOR_ARGS, data, reg ); \
        }, \
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) noexcept { \
            uint8_t value = 0; \
            if (retrieveByte(PASS_FILE_DESCRIPTOR_ARGS, data, size, value)) { \
                reg = value; \
            } \
        }, \
    }
#define WordFile(n, reg) { \
        .name = n , \
        .description = nullptr, \
        .help = nullptr, \
        .exec = nullptr, \
        .get_data = [](FILE_DESCRIPTOR_ARGS, uint8_t** data) { return sendWord(PASS_FILE_DESCRIPTOR_ARGS, data, reg ); }, \
        .set_data = [](FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size) {  \
            uint16_t result = 0; \
            if (retrieveWord(PASS_FILE_DESCRIPTOR_ARGS, data, size, result)) { \
                reg = result; \
            } \
        } \
    }

#define NELEM(obj) (sizeof(obj) / sizeof(obj[0]))

unsigned int sendByte(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint8_t value) noexcept;
bool retrieveWord(FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size, uint16_t& result) noexcept;
bool retrieveByte(FILE_DESCRIPTOR_ARGS, uint8_t* data, size_t size, uint8_t& result) noexcept;
unsigned int sendDword(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint32_t value) noexcept;
unsigned int sendWord(FILE_DESCRIPTOR_ARGS, uint8_t** data, uint16_t value) noexcept;

#endif // EXPERIMENT_PLATFORM_CONCEPTS_H__

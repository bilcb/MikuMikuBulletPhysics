#pragma once
#include "types.h"
#include <cstdint>

namespace mmp::vmd {

struct ParseResult {
    bool ok() const { return error == nullptr; }
    VMDData data;
    const char* error = nullptr;
};

ParseResult parse_memory(const uint8_t* data, int size);
ParseResult parse_file(const char* path);

} // namespace mmp::vmd

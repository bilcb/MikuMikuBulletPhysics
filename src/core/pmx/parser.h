#pragma once
#include "types.h"
#include <cstdint>
#include <string>

namespace mmp::pmx {

struct ParseError {
    const char* message = nullptr;
};

template <typename T>
struct ParseResult {
    bool ok() const { return m_error.message == nullptr; }
    const T& value() const { return m_value; }
    const ParseError& error() const { return m_error; }
    T& value() { return m_value; }
    ParseError m_error;
    T m_value;
};

ParseResult<PMXData> parse_file(const char* path);
ParseResult<PMXData> parse_memory(const uint8_t* data, int size);

} // namespace mmp::pmx

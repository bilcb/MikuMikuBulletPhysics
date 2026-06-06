#pragma once
#include "types.h"
#include <cstdint>
#include <string>

namespace mmbp::pmx {

struct ParseError {
  const char *message = nullptr;
};

template <typename T> struct ParseResult {
  bool ok() const { return m_error.message == nullptr; }
  const T &value() const { return m_value; }
  const ParseError &error() const { return m_error; }
  T &value() { return m_value; }
  ParseError m_error;
  T m_value;
};

namespace limits {
constexpr int32_t MAX_BONES = 65536;
constexpr int32_t MAX_RIGID_BODIES = 65536;
constexpr int32_t MAX_JOINTS = 65536;
constexpr int32_t MAX_MORPHS = 65536;
constexpr int32_t MAX_IK_LINKS = 256;
} // namespace limits

ParseResult<PMXData> parse_file(const char *path);
ParseResult<PMXData> parse_memory(const uint8_t *data, int size);

} // namespace mmbp::pmx

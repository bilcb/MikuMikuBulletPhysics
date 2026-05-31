#pragma once
#include <cstdint>
#include <string>

namespace mmp::util {

// Decode PMX string bytes to UTF-8.
// If declaredEncoding is correct, uses it directly.
// If encoding=0 (UTF-16LE) but decoded result looks wrong,
// falls back to UTF-8 (fixes mislabeled PMX files).
std::string decodePmxString(const uint8_t* data, int len, int declaredEncoding);

} // namespace mmp::util

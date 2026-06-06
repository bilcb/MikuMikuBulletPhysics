#pragma once
#include <cstdint>
#include <string>

namespace mmbp::util {

std::string decodePmxString(const uint8_t *data, int len, int declaredEncoding);

void resetEncodingCache();

std::string tryUtf16leToUtf8(const uint8_t *data, int len);

} // namespace mmbp::util

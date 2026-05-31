#include "encoding.h"
#include <cstring>

namespace mmp::util {

// Try UTF-16LE → UTF-8. Returns empty string on failure.
static std::string tryUtf16leToUtf8(const uint8_t* data, int len) {
    if (len < 2 || len % 2 != 0) return {};
    std::string result;
    result.reserve(len);
    for (int i = 0; i + 1 < len; i += 2) {
        uint16_t cp = data[i] | (uint16_t(data[i + 1]) << 8);
        // Surrogate pair
        if (cp >= 0xD800 && cp <= 0xDBFF) {
            if (i + 3 >= len) return {}; // incomplete pair
            uint16_t lo = data[i + 2] | (uint16_t(data[i + 3]) << 8);
            if (lo < 0xDC00 || lo > 0xDFFF) return {}; // invalid pair
            uint32_t full = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
            result += (char)(0xF0 | (full >> 18));
            result += (char)(0x80 | ((full >> 12) & 0x3F));
            result += (char)(0x80 | ((full >> 6) & 0x3F));
            result += (char)(0x80 | (full & 0x3F));
            i += 2;
        } else if (cp < 0x80) {
            result += (char)cp;
        } else if (cp < 0x800) {
            result += (char)(0xC0 | (cp >> 6));
            result += (char)(0x80 | (cp & 0x3F));
        } else {
            result += (char)(0xE0 | (cp >> 12));
            result += (char)(0x80 | ((cp >> 6) & 0x3F));
            result += (char)(0x80 | (cp & 0x3F));
        }
    }
    return result;
}

// Validate that a UTF-8 string looks like reasonable text for MMD names.
// Checks: no null bytes, no U+FFFD replacement chars, no control chars (except common ones).
static bool isValidMmdString(const std::string& s) {
    if (s.empty()) return false;
    int i = 0;
    int len = (int)s.size();
    while (i < len) {
        uint8_t b = (uint8_t)s[i];
        if (b == 0) return false; // null byte
        if (b < 0x80) {
            // ASCII: reject control chars except tab/newline
            if (b < 0x20 && b != '\t' && b != '\n' && b != '\r') return false;
            i++;
        } else if (b >= 0xC2 && b < 0xE0) {
            if (i + 1 >= len) return false;
            i += 2;
        } else if (b >= 0xE0 && b < 0xF0) {
            if (i + 2 >= len) return false;
            // Reject U+FFFD (EF BF BD) — replacement character
            if ((uint8_t)s[i] == 0xEF && (uint8_t)s[i+1] == 0xBF && (uint8_t)s[i+2] == 0xBD)
                return false;
            i += 3;
        } else if (b >= 0xF0 && b < 0xF8) {
            if (i + 3 >= len) return false;
            i += 4;
        } else {
            return false; // invalid lead byte
        }
    }
    return true;
}

std::string decodePmxString(const uint8_t* data, int len, int declaredEncoding) {
    if (!data || len <= 0) return {};

    if (declaredEncoding == 1) {
        // Declared UTF-8, trust it
        return std::string(reinterpret_cast<const char*>(data), len);
    }

    // Declared UTF-16LE (encoding=0)
    // Try UTF-16LE first
    std::string result = tryUtf16leToUtf8(data, len);

    // Validate: does the decoded result look like reasonable text?
    if (!result.empty() && isValidMmdString(result)) {
        return result;
    }

    // UTF-16LE decode produced garbage — fall back to UTF-8
    // (mislabeled by Chinese/Korean PMX exporters)
    return std::string(reinterpret_cast<const char*>(data), len);
}

} // namespace mmp::util

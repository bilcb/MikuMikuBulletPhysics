#include "encoding.h"
#include <cstring>

namespace mmbp::util {

struct EncodingCache {
  int lastEncoding = -1;
  bool valid = false;
};
thread_local EncodingCache tlsEncodingCache;

std::string tryUtf16leToUtf8(const uint8_t *data, int len) {
  if (!data || len < 2 || len % 2 != 0)
    return {};
  std::string result;
  result.reserve(len);
  for (int i = 0; i + 1 < len; i += 2) {
    uint16_t cp = data[i] | (uint16_t(data[i + 1]) << 8);

    if (cp >= 0xDC00 && cp <= 0xDFFF) {

      result += (char)0xEF;
      result += (char)0xBF;
      result += (char)0xBD;
      continue;
    }

    if (cp >= 0xD800 && cp <= 0xDBFF) {
      if (i + 3 >= len) {

        result += (char)0xEF;
        result += (char)0xBF;
        result += (char)0xBD;
        break;
      }
      uint16_t lo = data[i + 2] | (uint16_t(data[i + 3]) << 8);
      if (lo < 0xDC00 || lo > 0xDFFF) {

        result += (char)0xEF;
        result += (char)0xBF;
        result += (char)0xBD;
        continue;
      }
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

static bool isValidMmdString(const std::string &s) {
  if (s.empty())
    return false;
  int i = 0;
  int len = (int)s.size();
  while (i < len) {
    uint8_t b = (uint8_t)s[i];
    if (b == 0)
      return false;
    if (b < 0x80) {
      if (b < 0x20 && b != '\t' && b != '\n' && b != '\r')
        return false;
      i++;
    } else if (b >= 0xC2 && b <= 0xDF) {
      if (i + 1 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xC0) != 0x80)
        return false;
      i += 2;
    } else if (b == 0xE0) {
      if (i + 2 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xE0) != 0xA0)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      i += 3;
    } else if (b >= 0xE1 && b <= 0xEC) {
      if (i + 2 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xC0) != 0x80)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      i += 3;
    } else if (b == 0xED) {
      if (i + 2 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xE0) != 0x80)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      i += 3;
    } else if (b >= 0xEE && b <= 0xEF) {
      if (i + 2 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xC0) != 0x80)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      i += 3;
    } else if (b == 0xF0) {
      if (i + 3 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xF0) != 0x90)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      if (((uint8_t)s[i + 3] & 0xC0) != 0x80)
        return false;
      i += 4;
    } else if (b >= 0xF1 && b <= 0xF3) {
      if (i + 3 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xC0) != 0x80)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      if (((uint8_t)s[i + 3] & 0xC0) != 0x80)
        return false;
      i += 4;
    } else if (b == 0xF4) {
      if (i + 3 >= len)
        return false;
      if (((uint8_t)s[i + 1] & 0xF0) != 0x80)
        return false;
      if (((uint8_t)s[i + 2] & 0xC0) != 0x80)
        return false;
      if (((uint8_t)s[i + 3] & 0xC0) != 0x80)
        return false;
      i += 4;
    } else {
      return false;
    }
  }
  return true;
}

std::string decodePmxString(const uint8_t *data, int len,
                            int declaredEncoding) {
  if (!data || len <= 0)
    return {};

  if (tlsEncodingCache.valid &&
      tlsEncodingCache.lastEncoding == declaredEncoding) {

    std::string cached;
    if (declaredEncoding == 1) {
      cached.assign(reinterpret_cast<const char *>(data), len);
      if (isValidMmdString(cached))
        return cached;
    } else {
      cached = tryUtf16leToUtf8(data, len);
      if (!cached.empty() && isValidMmdString(cached))
        return cached;
    }

    tlsEncodingCache.valid = false;
  }

  if (declaredEncoding == 1) {

    std::string result(reinterpret_cast<const char *>(data), len);
    if (isValidMmdString(result)) {
      tlsEncodingCache = {declaredEncoding, true};
      return result;
    }

    std::string alt = tryUtf16leToUtf8(data, len);
    if (!alt.empty() && isValidMmdString(alt)) {
      tlsEncodingCache = {0, true};
      return alt;
    }

    return result;
  }

  std::string result = tryUtf16leToUtf8(data, len);
  if (!result.empty() && isValidMmdString(result)) {
    tlsEncodingCache = {declaredEncoding, true};
    return result;
  }

  tlsEncodingCache = {1, true};
  return std::string(reinterpret_cast<const char *>(data), len);
}

void resetEncodingCache() { tlsEncodingCache = {-1, false}; }

} // namespace mmbp::util

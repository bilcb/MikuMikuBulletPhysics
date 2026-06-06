#include "parser.h"
#include "core/util/encoding.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace mmbp::vpd {

#ifdef _WIN32
static std::string sjisToUtf8(const std::string &s) {
  if (s.empty())
    return s;
  int wlen = MultiByteToWideChar(932, 0, s.c_str(), (int)s.size(), nullptr, 0);
  if (wlen <= 0)
    return s;
  std::wstring wide(wlen, L'\0');
  MultiByteToWideChar(932, 0, s.c_str(), (int)s.size(), &wide[0], wlen);
  int ulen = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), wlen, nullptr, 0,
                                 nullptr, nullptr);
  if (ulen <= 0)
    return s;
  std::string utf8(ulen, '\0');
  WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), wlen, &utf8[0], ulen, nullptr,
                      nullptr);
  return utf8;
}
#else
#include <iconv.h>
static std::string sjisToUtf8(const std::string &s) {
  if (s.empty())
    return s;
  static thread_local struct IconvGuard {
    iconv_t cd;
    IconvGuard() {
      cd = iconv_open("UTF-8", "SHIFT_JIS");
      if (cd == (iconv_t)-1)
        cd = iconv_open("UTF-8", "CP932");
    }
    ~IconvGuard() {
      if (cd != (iconv_t)-1)
        iconv_close(cd);
    }
  } guard;
  if (guard.cd == (iconv_t)-1)
    return s;
  size_t inLeft = s.size();
  size_t outLeft = s.size() * 4;
  std::string result(outLeft, '\0');
  char *inPtr = const_cast<char *>(s.data());
  char *outPtr = &result[0];
  iconv(guard.cd, nullptr, nullptr, nullptr, nullptr);
  iconv(guard.cd, &inPtr, &inLeft, &outPtr, &outLeft);
  result.resize(outPtr - &result[0]);
  return result;
}
#endif

static std::string decodeToUtf8(const uint8_t *data, int size) {
  if (!data || size <= 0)
    return {};

  if (size >= 2) {
    if (data[0] == 0xFF && data[1] == 0xFE) {
      int off = 2;
      if (size >= 4 && data[2] == 0x00 && data[3] == 0x00)
        off = 4;
      return util::tryUtf16leToUtf8(data + off, size - off);
    }
    if (data[0] == 0xFE && data[1] == 0xFF) {
      std::vector<uint8_t> le(static_cast<size_t>(size));
      for (int i = 2; i + 1 < size; i += 2) {
        le[i] = data[i + 1];
        le[i + 1] = data[i];
      }
      return util::tryUtf16leToUtf8(le.data() + 2, size - 2);
    }
    if (size >= 3 && data[0] == 0xEF && data[1] == 0xBB && data[2] == 0xBF)
      return std::string(reinterpret_cast<const char *>(data + 3), size - 3);
  }

  return sjisToUtf8(std::string(reinterpret_cast<const char *>(data), size));
}

static std::string trim(const std::string &s) {
  size_t start = 0;
  while (start < s.size() &&
         (s[start] == ' ' || s[start] == '\t' || s[start] == '\r'))
    start++;
  if (start >= s.size())
    return {};
  size_t end = s.size() - 1;
  while (end > start &&
         (s[end] == ' ' || s[end] == '\t' || s[end] == '\r' || s[end] == '\n'))
    end--;
  return s.substr(start, end - start + 1);
}

static std::vector<std::string> splitLines(const std::string &text) {
  std::vector<std::string> lines;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line))
    lines.push_back(line);
  return lines;
}

static std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> parts;
  std::istringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
    parts.push_back(item);
  return parts;
}

static void skipEmptyLines(const std::vector<std::string> &lines, size_t &idx) {
  while (idx < lines.size() && trim(lines[idx]).empty())
    idx++;
}

static bool parseFloat4(const std::string &str, float *out) {
  auto parts = split(str, ',');
  if (parts.size() < 4)
    return false;
  try {
    for (int i = 0; i < 4; i++)
      out[i] = std::stof(trim(parts[i]));
    return true;
  } catch (...) {
    return false;
  }
}

static bool parseFloat3(const std::string &str, float *out) {
  auto parts = split(str, ',');
  if (parts.size() < 3)
    return false;
  try {
    for (int i = 0; i < 3; i++)
      out[i] = std::stof(trim(parts[i]));
    return true;
  } catch (...) {
    return false;
  }
}

VPDParseResult parse_memory(const uint8_t *data, int size) {
  VPDParseResult result;
  if (!data || size <= 0) {
    result.error = "VPD data is empty";
    return result;
  }

  std::string text = decodeToUtf8(data, size);
  if (text.empty()) {
    result.error = "Failed to decode VPD text";
    return result;
  }

  auto lines = splitLines(text);
  size_t li = 0;
  skipEmptyLines(lines, li);

  if (li >= lines.size()) {
    result.error = "VPD file is empty";
    return result;
  }

  std::string header = trim(lines[li++]);
  if (header != "Vocaloid Pose Data file") {
    result.error = "Invalid VPD header";
    return result;
  }

  skipEmptyLines(lines, li);
  if (li >= lines.size()) {
    result.error = "Missing bone count";
    return result;
  }

  int boneCount = 0;
  try {
    boneCount = std::stoi(trim(lines[li++]));
  } catch (...) {
    result.error = "Invalid bone count";
    return result;
  }

  for (int i = 0; i < boneCount; i++) {
    skipEmptyLines(lines, li);
    if (li >= lines.size())
      break;

    VPDPoseBone bone;

    std::string body;
    size_t bracePos = lines[li].find('{');
    if (bracePos == std::string::npos) {
      bone.name = trim(lines[li++]);
      if (bone.name.empty())
        continue;
      while (li < lines.size()) {
        std::string l = lines[li++];
        size_t cp = l.find('}');
        if (cp != std::string::npos) {
          body += l.substr(0, cp);
          break;
        }
        body += l;
      }
    } else {
      bone.name = trim(lines[li].substr(0, bracePos));
      body = lines[li].substr(bracePos + 1);
      li++;
      size_t closePos = body.rfind('}');
      if (closePos != std::string::npos) {
        body = body.substr(0, closePos);
      } else {
        while (li < lines.size()) {
          std::string l = lines[li++];
          size_t cp = l.find('}');
          if (cp != std::string::npos) {
            body += l.substr(0, cp);
            break;
          }
          body += l;
        }
      }
    }

    size_t rotPos = body.find("rotation(");
    if (rotPos != std::string::npos) {
      size_t end = body.find(')', rotPos);
      if (end != std::string::npos) {
        std::string vals = body.substr(rotPos + 9, end - rotPos - 9);
        parseFloat4(vals, bone.rotation);
      }
    }

    size_t posPos = body.find("position(");
    if (posPos != std::string::npos) {
      size_t end = body.find(')', posPos);
      if (end != std::string::npos) {
        std::string vals = body.substr(posPos + 9, end - posPos - 9);
        parseFloat3(vals, bone.position);
      }
    }

    for (int j = 0; j < 3; j++) {
      if (!std::isfinite(bone.position[j])) {
        bone.position[j] = 0;
      }
    }
    for (int j = 0; j < 4; j++) {
      if (!std::isfinite(bone.rotation[j])) {
        bone.rotation[j] = (j == 3 ? 1.0f : 0.0f);
      }
    }

    result.data.bones.push_back(std::move(bone));
  }

  skipEmptyLines(lines, li);
  if (li >= lines.size())
    return result;

  int morphCount = 0;
  try {
    morphCount = std::stoi(trim(lines[li++]));
  } catch (...) {
    result.error = "Invalid morph count";
    return result;
  }

  for (int i = 0; i < morphCount; i++) {
    skipEmptyLines(lines, li);
    if (li >= lines.size())
      break;

    VPDPoseMorph morph;

    size_t bracePos = lines[li].find('{');
    std::string weightStr;
    if (bracePos == std::string::npos) {
      morph.name = trim(lines[li++]);
      if (morph.name.empty())
        continue;
      while (li < lines.size()) {
        std::string l = lines[li++];
        size_t cp = l.find('}');
        if (cp != std::string::npos) {
          weightStr = l.substr(0, cp);
          break;
        }
        weightStr = l;
      }
    } else {
      morph.name = trim(lines[li].substr(0, bracePos));
      std::string between = lines[li].substr(bracePos + 1);
      li++;
      size_t closePos = between.rfind('}');
      if (closePos != std::string::npos) {
        weightStr = between.substr(0, closePos);
      } else {
        weightStr = between;
        while (li < lines.size()) {
          std::string l = lines[li++];
          size_t cp = l.find('}');
          if (cp != std::string::npos) {
            weightStr += l.substr(0, cp);
            break;
          }
          weightStr += l;
        }
      }
    }

    weightStr = trim(weightStr);
    if (!weightStr.empty() && weightStr.back() == ';')
      weightStr.pop_back();
    try {
      morph.weight = std::stof(weightStr);
    } catch (...) {
      morph.weight = 0.0f;
    }

    if (!std::isfinite(morph.weight))
      morph.weight = 0.0f;
    else if (morph.weight < 0.0f || morph.weight > 1.0f)
      morph.weight = std::max(0.0f, std::min(1.0f, morph.weight));

    result.data.morphs.push_back(std::move(morph));
  }

  return result;
}

VPDParseResult parse_file(const char *path) {
  VPDParseResult result;
#ifdef _WIN32
  int pathLen = MultiByteToWideChar(CP_UTF8, 0, path, -1, nullptr, 0);
  FILE *fp = nullptr;
  if (pathLen > 0) {
    std::wstring wpath(static_cast<size_t>(pathLen), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path, -1, &wpath[0], pathLen);
    fp = _wfopen(wpath.c_str(), L"rb");
  }
#else
  FILE *fp = fopen(path, "rb");
#endif
  if (!fp) {
    result.error = "Failed to open VPD file";
    return result;
  }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz <= 0) {
    fclose(fp);
    result.error = "VPD file is empty";
    return result;
  }
  std::vector<uint8_t> buffer(static_cast<size_t>(sz));
  if (fread(buffer.data(), 1, static_cast<size_t>(sz), fp) !=
      static_cast<size_t>(sz)) {
    fclose(fp);
    result.error = "Failed to read VPD file";
    return result;
  }
  fclose(fp);
  return parse_memory(buffer.data(), static_cast<int>(buffer.size()));
}

} // namespace mmbp::vpd

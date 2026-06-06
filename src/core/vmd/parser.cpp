#include "parser.h"
#include "core/anim/evaluator.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

namespace {

constexpr size_t kMaxWarnings = 1024;
constexpr const char *kWarningsTruncatedMarker =
    "[truncated: more than 1024 warnings, see server log for full list]";

inline void pushWarning(mmbp::vmd::VMDData &vmd, std::string &&msg) {
  if (vmd.warnings.size() < kMaxWarnings) {
    vmd.warnings.push_back(std::move(msg));
  } else if (vmd.warnings.size() == kMaxWarnings) {
    vmd.warnings.emplace_back(kWarningsTruncatedMarker);
  }
}
} // namespace

#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif

namespace mmbp::vmd {

#ifndef _WIN32
struct IconvGuard {
  iconv_t cd = (iconv_t)-1;
  IconvGuard() {
    cd = iconv_open("UTF-8", "SHIFT_JIS");
    if (cd == (iconv_t)-1)
      cd = iconv_open("UTF-8", "CP932");
  }
  ~IconvGuard() {
    if (cd != (iconv_t)-1)
      iconv_close(cd);
  }
  IconvGuard(const IconvGuard &) = delete;
  IconvGuard &operator=(const IconvGuard &) = delete;
};
#endif

static std::pair<std::string, bool> sjisToUtf8WithFlag(std::string sjis) {
  if (sjis.empty())
    return {sjis, false};

  if (sjis.size() > 1) {
    size_t nullPos = sjis.find('\0', 1);
    if (nullPos != std::string::npos)
      sjis.resize(nullPos);
  }

  bool hasErrorFlag = (sjis[0] == '\0');
  std::string toDecode;
  if (hasErrorFlag) {
    toDecode = sjis.substr(1);
  } else {
    toDecode = sjis;
  }

  auto decodeSjis = [](const std::string &s) -> std::string {
#ifdef _WIN32
    if (s.empty())
      return s;
    int wlen =
        MultiByteToWideChar(932, 0, s.c_str(), (int)s.size(), nullptr, 0);
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
#else
    if (s.empty())
      return s;
    thread_local IconvGuard guard;
    if (guard.cd == (iconv_t)-1)
      return s;
    iconv(guard.cd, nullptr, nullptr, nullptr, nullptr);
    size_t inLeft = s.size();
    size_t outLeft = s.size() * 4;
    std::string result(outLeft, '\0');
    char *inPtr = const_cast<char *>(s.data());
    char *outPtr = &result[0];

    size_t ret = iconv(guard.cd, &inPtr, &inLeft, &outPtr, &outLeft);
    if (ret == (size_t)-1) {
    }
    result.resize(outPtr - &result[0]);
    return result;
#endif
  };

  std::string decoded = decodeSjis(toDecode);

  if (hasErrorFlag) {
    std::string result;
    result.reserve(decoded.size() + 6);
    for (auto c : decoded) {
      if (c == '?')
        result.append("\xef\xbf\xbd");
      else
        result.push_back(c);
    }
    return {std::string("\xef\xbf\xbd") + result, true};
  }
  return {decoded, false};
}

static std::string sjisToUtf8(std::string sjis) {
  return sjisToUtf8WithFlag(std::move(sjis)).first;
}

static const char MAGIC[30] = "Vocaloid Motion Data 0002";
static const int MAGIC_SIZE = 30;
static const int MODEL_NAME_SIZE = 20;
static const int BONE_NAME_SIZE = 15;

ParseResult parse_memory(const uint8_t *data, int size) {
  ParseResult result;
  if (!data || size < MAGIC_SIZE) {
    result.error = "VMD data too small: need at least 30 bytes for header";
    return result;
  }
  if (std::memcmp(data, "Vocaloid Motion Data 0002", 25) != 0) {
    result.error = "Invalid VMD magic: expected 'Vocaloid Motion Data 0002'";
    return result;
  }
  int pos = MAGIC_SIZE;

  if (pos + MODEL_NAME_SIZE > size) {
    result.error = "data too small for model name";
    return result;
  }
  result.data.name = sjisToUtf8(
      std::string(reinterpret_cast<const char *>(&data[pos]), MODEL_NAME_SIZE));
  pos += MODEL_NAME_SIZE;

  if (pos + 4 > size) {
    result.error = "data too small for bone keyframe count";
    return result;
  }
  uint32_t boneCount;
  std::memcpy(&boneCount, &data[pos], sizeof(boneCount));
  pos += 4;

  for (uint32_t i = 0; i < boneCount; i++) {
    if (pos + BONE_NAME_SIZE + 4 + 12 + 16 + 64 > size) {
      result.error = "data too small for bone keyframe";
      return result;
    }

    auto [boneName, boneEncErr] = sjisToUtf8WithFlag(std::string(
        reinterpret_cast<const char *>(&data[pos]), BONE_NAME_SIZE));
    if (boneEncErr) {
      pushWarning(result.data, "Bone keyframe " + std::to_string(i) +
                                   ": encoding error in name '" + boneName +
                                   "'");
    }
    VMDKeyframe kf;
    kf.boneName = std::move(boneName);
    pos += BONE_NAME_SIZE;

    std::memcpy(&kf.frame, &data[pos], sizeof(kf.frame));
    pos += 4;

    std::memcpy(&kf.translation, &data[pos], 12);
    pos += 12;

    std::memcpy(&kf.rotation, &data[pos], 16);
    pos += 16;

    constexpr float Q_EPS = 1e-8f;
    if (std::abs(kf.rotation.w()) < Q_EPS &&
        std::abs(kf.rotation.x()) < Q_EPS &&
        std::abs(kf.rotation.y()) < Q_EPS && std::abs(kf.rotation.z()) < Q_EPS)
      kf.rotation = btQuaternion(0, 0, 0, 1);

    std::memcpy(kf.interpX, &data[pos], 16);
    pos += 16;
    std::memcpy(kf.interpY, &data[pos], 16);
    pos += 16;
    std::memcpy(kf.interpZ, &data[pos], 16);
    pos += 16;
    std::memcpy(kf.interpRot, &data[pos], 16);
    pos += 16;

    if (!std::isfinite(kf.translation.x()) ||
        !std::isfinite(kf.translation.y()) ||
        !std::isfinite(kf.translation.z())) {
      pushWarning(
          result.data,
          "Bone keyframe " + std::to_string(i) + " ('" + kf.boneName +
              "'): translation contains non-finite value(s), clamping to 0");
      kf.translation = btVector3(0, 0, 0);
    }
    if (!std::isfinite(kf.rotation.x()) || !std::isfinite(kf.rotation.y()) ||
        !std::isfinite(kf.rotation.z()) || !std::isfinite(kf.rotation.w())) {
      pushWarning(result.data, "Bone keyframe " + std::to_string(i) + " ('" +
                                   kf.boneName +
                                   "'): rotation contains non-finite value(s), "
                                   "resetting to identity");
      kf.rotation = btQuaternion(0, 0, 0, 1);
    }

    result.data.keyframes.push_back(std::move(kf));
  }

  std::sort(result.data.keyframes.begin(), result.data.keyframes.end(),
            [](const VMDKeyframe &a, const VMDKeyframe &b) {
              return a.frame < b.frame;
            });

  if (pos + 4 > size) {
    (void)pos;
    return result;
  }
  uint32_t morphCount;
  std::memcpy(&morphCount, &data[pos], sizeof(morphCount));
  pos += 4;

  for (uint32_t i = 0; i < morphCount; i++) {
    if (pos + BONE_NAME_SIZE + 4 + 4 > size)
      break;
    MorphKeyframe mk;

    auto [morphName, morphEncErr] = sjisToUtf8WithFlag(std::string(
        reinterpret_cast<const char *>(&data[pos]), BONE_NAME_SIZE));
    if (morphEncErr) {
      pushWarning(result.data, "Morph keyframe " + std::to_string(i) +
                                   ": encoding error in name '" + morphName +
                                   "'");
    }
    mk.name = std::move(morphName);
    pos += BONE_NAME_SIZE;

    std::memcpy(&mk.frame, &data[pos], sizeof(mk.frame));
    pos += 4;

    std::memcpy(&mk.weight, &data[pos], sizeof(mk.weight));
    pos += 4;

    if (!std::isfinite(mk.weight)) {
      pushWarning(result.data, "Morph keyframe " + std::to_string(i) + " ('" +
                                   mk.name +
                                   "'): weight is non-finite, clamping to 0");
      mk.weight = 0.0f;
    } else if (mk.weight < 0.0f || mk.weight > 1.0f) {
      pushWarning(result.data, "Morph keyframe " + std::to_string(i) + " ('" +
                                   mk.name +
                                   "'): weight = " + std::to_string(mk.weight) +
                                   " out of [0,1] range");
      mk.weight = std::max(0.0f, std::min(1.0f, mk.weight));
    }

    result.data.morphKeys.push_back(std::move(mk));
  }

  std::sort(result.data.morphKeys.begin(), result.data.morphKeys.end(),
            [](const MorphKeyframe &a, const MorphKeyframe &b) {
              return a.frame < b.frame;
            });

  if (pos + 4 <= size) {
    uint32_t cameraCount;
    std::memcpy(&cameraCount, &data[pos], sizeof(cameraCount));
    pos += 4;
    if (cameraCount > 0) {
      int64_t camSkip = static_cast<int64_t>(cameraCount) * 61;
      if (pos + camSkip > size) {
        pos = size;
      } else {
        pos += static_cast<int>(camSkip);
      }
    }
  }

  if (pos + 4 <= size) {
    uint32_t lightCount;
    std::memcpy(&lightCount, &data[pos], sizeof(lightCount));
    pos += 4;
    if (lightCount > 0) {
      int64_t lightSkip = static_cast<int64_t>(lightCount) * 28;
      if (pos + lightSkip > size) {
        pos = size;
      } else {
        pos += static_cast<int>(lightSkip);
      }
    }
  }

  if (pos + 4 <= size) {
    uint32_t shadowCount;
    std::memcpy(&shadowCount, &data[pos], sizeof(shadowCount));
    pos += 4;
    for (uint32_t i = 0; i < shadowCount && pos + 9 <= size; i++) {
      ShadowKeyframe sk;
      std::memcpy(&sk.frame, &data[pos], sizeof(sk.frame));
      pos += 4;
      std::memcpy(&sk.mode, &data[pos], 1);
      pos += 1;
      uint32_t rawDist;
      std::memcpy(&rawDist, &data[pos], sizeof(rawDist));
      pos += 4;
      sk.distance = 10000.0f - static_cast<float>(rawDist) * 100000.0f;
      result.data.shadowKeys.push_back(sk);
    }
  }

  if (pos + 4 <= size) {
    uint32_t ikCount;
    std::memcpy(&ikCount, &data[pos], sizeof(ikCount));
    pos += 4;
    for (uint32_t i = 0; i < ikCount && pos + 5 <= size; i++) {
      IKKeyframe ik;
      std::memcpy(&ik.frame, &data[pos], sizeof(ik.frame));
      pos += 4;
      uint8_t show;
      std::memcpy(&show, &data[pos], 1);
      pos += 1;
      ik.show = (show != 0);
      if (pos + 4 <= size) {
        uint32_t ikInfoCount;
        std::memcpy(&ikInfoCount, &data[pos], sizeof(ikInfoCount));
        pos += 4;
        for (uint32_t j = 0; j < ikInfoCount && pos + 21 <= size; j++) {
          IKKeyframe::IKState st;

          auto [ikName, ikEncErr] = sjisToUtf8WithFlag(std::string(
              reinterpret_cast<const char *>(&data[pos]), BONE_NAME_SIZE));
          if (ikEncErr) {
            pushWarning(result.data, "IK keyframe " + std::to_string(i) +
                                         ": encoding error in name '" + ikName +
                                         "'");
          }
          st.name = std::move(ikName);

          pos += 20;
          uint8_t en;
          std::memcpy(&en, &data[pos], 1);
          pos += 1;
          st.enabled = (en != 0);
          ik.states.push_back(st);
        }
      }
      result.data.ikKeys.push_back(std::move(ik));
    }
  }

  std::sort(result.data.ikKeys.begin(), result.data.ikKeys.end(),
            [](const IKKeyframe &a, const IKKeyframe &b) {
              return a.frame < b.frame;
            });

  mmbp::anim::groupKeyframesByBone(result.data);
  mmbp::anim::groupMorphKeyframes(result.data);

  (void)pos;
  return result;
}

ParseResult parse_file(const char *path) {
  ParseResult result;
#ifdef _WIN32
  int pathLen = MultiByteToWideChar(CP_UTF8, 0, path, -1, nullptr, 0);
  FILE *fp = nullptr;
  if (pathLen > 0) {
    std::wstring wpath(pathLen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path, -1, &wpath[0], pathLen);
    fp = _wfopen(wpath.c_str(), L"rb");
  }
#else
  FILE *fp = fopen(path, "rb");
#endif
  if (!fp) {
    result.error = "Failed to open VMD file";
    return result;
  }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz <= 0) {
    fclose(fp);
    result.error = "VMD file is empty";
    return result;
  }
  std::vector<uint8_t> buffer(static_cast<size_t>(sz));
  if (fread(buffer.data(), 1, static_cast<size_t>(sz), fp) !=
      static_cast<size_t>(sz)) {
    fclose(fp);
    result.error = "Failed to read VMD file";
    return result;
  }
  fclose(fp);
  return parse_memory(buffer.data(), static_cast<int>(buffer.size()));
}

} // namespace mmbp::vmd

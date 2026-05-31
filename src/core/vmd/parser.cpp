#include "parser.h"
#include <cstring>
#include <fstream>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif

namespace mmp::vmd {

// Convert Shift-JIS bytes to UTF-8, handling MMD encoding error flag.
// If the first byte is \x00, it indicates an encoding failure in MMD.
// We prepend U+FFFD and replace '?' with U+FFFD for consistency.
static std::string sjisToUtf8(std::string sjis) {
    if (sjis.empty()) return sjis;

    // Truncate at null terminator (skip first byte) to remove trailing padding
    if (sjis.size() > 1) {
        size_t nullPos = sjis.find('\0', 1);
        if (nullPos != std::string::npos) sjis.resize(nullPos);
    }

    bool hasErrorFlag = (sjis[0] == '\0');
    std::string toDecode;
    if (hasErrorFlag) {
        // Skip the error flag byte, decode the rest
        toDecode = sjis.substr(1);
    } else {
        toDecode = sjis;
    }

    auto decodeSjis = [](const std::string& s) -> std::string {
#ifdef _WIN32
        if (s.empty()) return s;
        int wlen = MultiByteToWideChar(932, 0, s.c_str(), (int)s.size(), nullptr, 0);
        if (wlen <= 0) return s;
        std::wstring wide(wlen, L'\0');
        MultiByteToWideChar(932, 0, s.c_str(), (int)s.size(), &wide[0], wlen);
        int ulen = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), wlen, nullptr, 0, nullptr, nullptr);
        if (ulen <= 0) return s;
        std::string utf8(ulen, '\0');
        WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), wlen, &utf8[0], ulen, nullptr, nullptr);
        return utf8;
#else
        if (s.empty()) return s;
        static iconv_t cd = (iconv_t)-1;
        static bool tried = false;
        if (!tried) {
            tried = true;
            cd = iconv_open("UTF-8", "SHIFT_JIS");
            if (cd == (iconv_t)-1)
                cd = iconv_open("UTF-8", "CP932");
        }
        if (cd == (iconv_t)-1) return s;
        iconv(cd, nullptr, nullptr, nullptr, nullptr);
        size_t inLeft = s.size();
        size_t outLeft = s.size() * 4;
        std::string result(outLeft, '\0');
        char* inPtr = const_cast<char*>(s.data());
        char* outPtr = &result[0];
        size_t ret = iconv(cd, &inPtr, &inLeft, &outPtr, &outLeft);
        result.resize(outPtr - &result[0]);
        return result;
#endif
    };

    std::string decoded = decodeSjis(toDecode);

    if (hasErrorFlag) {
        // Replace '?' with U+FFFD for consistency with MMD behavior,
        // and prepend U+FFFD to indicate the first character was masked
        for (auto& c : decoded) {
            if (c == '?') c = '\xef';  // partial U+FFFD, will be handled below
        }
        // Prepend U+FFFD (ef bf bd)
        return std::string("\xef\xbf\xbd") + decoded;
    }
    return decoded;
}

static const char MAGIC[30] = "Vocaloid Motion Data 0002";
static const int MAGIC_SIZE = 30;
static const int MODEL_NAME_SIZE = 20;
static const int BONE_NAME_SIZE = 15;

ParseResult parse_memory(const uint8_t* data, int size) {
    ParseResult result;
    if (!data || size < MAGIC_SIZE) {
        result.error = "data too small";
        return result;
    }
    if (std::memcmp(data, "Vocaloid Motion Data 0002", 25) != 0) {
        result.error = "invalid magic";
        return result;
    }
    int pos = MAGIC_SIZE;

    if (pos + MODEL_NAME_SIZE > size) {
        result.error = "data too small for model name";
        return result;
    }
    result.data.name = sjisToUtf8(std::string(reinterpret_cast<const char*>(&data[pos]), MODEL_NAME_SIZE));
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
        VMDKeyframe kf;
        kf.boneName = sjisToUtf8(std::string(reinterpret_cast<const char*>(&data[pos]), BONE_NAME_SIZE));
        pos += BONE_NAME_SIZE;

        std::memcpy(&kf.frame, &data[pos], sizeof(kf.frame));
        pos += 4;

        std::memcpy(&kf.translation, &data[pos], 12);
        pos += 12;

        std::memcpy(&kf.rotation, &data[pos], 16);
        pos += 16;
        // Guard against zero quaternion from buggy exporters
        if (kf.rotation.x()==0 && kf.rotation.y()==0 && kf.rotation.z()==0 && kf.rotation.w()==0)
            kf.rotation = btQuaternion(0,0,0,1);

        std::memcpy(kf.interpX, &data[pos], 16); pos += 16;
        std::memcpy(kf.interpY, &data[pos], 16); pos += 16;
        std::memcpy(kf.interpZ, &data[pos], 16); pos += 16;
        std::memcpy(kf.interpRot, &data[pos], 16); pos += 16;

        result.data.keyframes.push_back(std::move(kf));
    }

    if (pos + 4 > size) { (void)pos; return result; }  // no morph data, success
    uint32_t morphCount;
    std::memcpy(&morphCount, &data[pos], sizeof(morphCount));
    pos += 4;

    for (uint32_t i = 0; i < morphCount; i++) {
        if (pos + BONE_NAME_SIZE + 4 + 4 > size) break;
        MorphKeyframe mk;
        mk.name = sjisToUtf8(std::string(reinterpret_cast<const char*>(&data[pos]), BONE_NAME_SIZE));
        pos += BONE_NAME_SIZE;

        std::memcpy(&mk.frame, &data[pos], sizeof(mk.frame));
        pos += 4;

        std::memcpy(&mk.weight, &data[pos], sizeof(mk.weight));
        pos += 4;

        result.data.morphKeys.push_back(std::move(mk));
    }

    // camera keyframe count (discard)
    if (pos + 4 <= size) {
        uint32_t cameraCount;
        std::memcpy(&cameraCount, &data[pos], sizeof(cameraCount));
        pos += 4;
        // skip camera keyframes
        if (cameraCount > 0) {
            // camera keyframe = 4(frame) + 4(distance) + 12(position) + 12(rotation) + 24(interp) + 4(viewAngle) + 1(perspective) = 61
            int camSkip = static_cast<int>(cameraCount) * 61;
            if (pos + camSkip > size) { pos = size; } else { pos += camSkip; }
        }
    }

    // light keyframe count (discard)
    if (pos + 4 <= size) {
        uint32_t lightCount;
        std::memcpy(&lightCount, &data[pos], sizeof(lightCount));
        pos += 4;
        if (lightCount > 0) {
            // light keyframe = 4(frame) + 12(rgb) + 12(position) = 28
            int lightSkip = static_cast<int>(lightCount) * 28;
            if (pos + lightSkip > size) { pos = size; } else { pos += lightSkip; }
        }
    }

    // shadow keyframe count (discard)
    if (pos + 4 <= size) {
        uint32_t shadowCount;
        std::memcpy(&shadowCount, &data[pos], sizeof(shadowCount));
        pos += 4;
        if (shadowCount > 0) {
            // shadow keyframe = 4(frame) + 1(mode) + 4(distance) = 9
            pos += static_cast<int>(shadowCount) * 9;
            if (pos > size) pos = size;
        }
    }

    // IK/property keyframe count (parse)
    if (pos + 4 <= size) {
        uint32_t ikCount;
        std::memcpy(&ikCount, &data[pos], sizeof(ikCount));
        pos += 4;
        for (uint32_t i = 0; i < ikCount && pos + 5 <= size; i++) {
            IKKeyframe ik;
            std::memcpy(&ik.frame, &data[pos], sizeof(ik.frame)); pos += 4;
            uint8_t show; std::memcpy(&show, &data[pos], 1); pos += 1;
            ik.show = (show != 0);
            if (pos + 4 <= size) {
                uint32_t ikInfoCount;
                std::memcpy(&ikInfoCount, &data[pos], sizeof(ikInfoCount)); pos += 4;
                for (uint32_t j = 0; j < ikInfoCount && pos + 21 <= size; j++) {
                    IKKeyframe::IKState st;
                    // VMD format: IK name is 20 bytes but only first 15 are valid
                    st.name = sjisToUtf8(std::string(reinterpret_cast<const char*>(&data[pos]), BONE_NAME_SIZE));
                    pos += 20;  // advance past full 20-byte field
                    uint8_t en; std::memcpy(&en, &data[pos], 1); pos += 1;
                    st.enabled = (en != 0);
                    ik.states.push_back(st);
                }
            }
            result.data.ikKeys.push_back(std::move(ik));
        }
    }

    (void)pos;
    return result;
}

ParseResult parse_file(const char* path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        ParseResult result;
        result.error = "failed to open file";
        return result;
    }
    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(static_cast<size_t>(fileSize));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize)) {
        ParseResult result;
        result.error = "failed to read file";
        return result;
    }
    return parse_memory(buffer.data(), static_cast<int>(buffer.size()));
}

} // namespace mmp::vmd

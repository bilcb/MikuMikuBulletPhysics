#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace mmbp::vpd {

struct VPDPoseBone {
  std::string name;
  float position[3] = {0, 0, 0};
  float rotation[4] = {1, 0, 0, 0};
};

struct VPDPoseMorph {
  std::string name;
  float weight = 0;
};

struct VPDData {
  std::vector<VPDPoseBone> bones;
  std::vector<VPDPoseMorph> morphs;
};

struct VPDParseResult {
  VPDData data;
  const char *error = nullptr;
  bool ok() const { return error == nullptr; }
};

VPDParseResult parse_memory(const uint8_t *data, int size);
VPDParseResult parse_file(const char *path);

} // namespace mmbp::vpd

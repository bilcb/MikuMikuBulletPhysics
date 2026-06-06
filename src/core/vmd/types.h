#pragma once
#include <btBulletDynamicsCommon.h>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace mmbp::vmd {

struct VMDKeyframe {
  std::string boneName;
  uint32_t frame = 0;
  btVector3 translation = btVector3(0, 0, 0);
  btQuaternion rotation = btQuaternion(0, 0, 0, 1);
  uint8_t interpX[16] = {0};
  uint8_t interpY[16] = {0};
  uint8_t interpZ[16] = {0};
  uint8_t interpRot[16] = {0};
};

struct MorphKeyframe {
  std::string name;
  uint32_t frame = 0;
  float weight = 0;
};

struct IKKeyframe {
  uint32_t frame = 0;
  bool show = true;
  struct IKState {
    std::string name;
    bool enabled = false;
  };
  std::vector<IKState> states;
};

struct ShadowKeyframe {
  uint32_t frame = 0;
  uint8_t mode = 0;
  float distance = 0.0f;
};

struct VMDData {
  std::string name;
  std::vector<VMDKeyframe> keyframes;
  std::vector<MorphKeyframe> morphKeys;
  std::vector<IKKeyframe> ikKeys;
  std::vector<ShadowKeyframe> shadowKeys;
  std::vector<std::string> warnings;

  std::unordered_map<std::string, std::vector<size_t>> boneKeyframeGroups;
  std::unordered_map<std::string, std::vector<size_t>> morphKeyframeGroups;
};

} // namespace mmbp::vmd

#pragma once
#include "core/pmx/types.h"
#include "types.h"
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace mmbp::model {

class Model {
public:
  bool load(const pmx::PMXData &pmxData);
  size_t boneCount() const { return m_bones.size(); }
  BoneNode *getBone(size_t idx) {
    return idx < m_bones.size() ? m_bones[idx].get() : nullptr;
  }
  const BoneNode *getBone(size_t idx) const {
    return idx < m_bones.size() ? m_bones[idx].get() : nullptr;
  }
  BoneNode *getBone(const std::string &name);
  const BoneNode *getBone(const std::string &name) const;
  void resetAnimation();
  void saveBaseAnimation();
  void loadBaseAnimation();
  int findMorphIndex(const std::string &name) const;
  float *getMorphWeightPtr(size_t idx);
  bool isTopoValid() const { return m_topologyVersion > 0; }
  size_t orphanBoneCount() const { return m_orphanBoneCount; }

  uint32_t topologyVersion() const { return m_topologyVersion; }

private:
  std::vector<std::unique_ptr<BoneNode>> m_bones;
  std::unordered_map<std::string, BoneNode *> m_boneMap;
  std::vector<float> m_morphWeights;
  std::vector<std::string> m_morphNames;
  std::unordered_map<std::string, int> m_morphMap;
  bool m_topoValid = false;
  uint32_t m_topologyVersion = 0;
  size_t m_orphanBoneCount = 0;
};

} // namespace mmbp::model

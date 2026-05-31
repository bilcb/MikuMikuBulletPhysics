#pragma once
#include "types.h"
#include "core/pmx/types.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

namespace mmp::model {

class Model {
public:
    bool load(const pmx::PMXData& pmxData);
    size_t boneCount() const { return m_bones.size(); }
    BoneNode* getBone(size_t idx) { return idx < m_bones.size() ? m_bones[idx].get() : nullptr; }
    const BoneNode* getBone(size_t idx) const { return idx < m_bones.size() ? m_bones[idx].get() : nullptr; }
    BoneNode* getBone(const std::string& name);
    const BoneNode* getBone(const std::string& name) const;
    void resetAnimation();
    void saveBaseAnimation();
    void loadBaseAnimation();
    int findMorphIndex(const std::string& name) const;
    float* getMorphWeightPtr(size_t idx);
    void computeInverseInitTransforms();

private:
    std::vector<std::unique_ptr<BoneNode>> m_bones;
    std::unordered_map<std::string, BoneNode*> m_boneMap;
    std::vector<float> m_morphWeights;
    std::vector<std::string> m_morphNames;
};

} // namespace mmp::model

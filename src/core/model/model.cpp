#include "model.h"

namespace mmp::model {

bool Model::load(const pmx::PMXData& pmxData) {
    m_bones.clear();
    m_boneMap.clear();
    m_morphWeights.clear();
    m_morphNames.clear();
    int n = static_cast<int>(pmxData.bones.size());
    m_bones.reserve(n);
    for (int i = 0; i < n; i++) {
        auto node = std::make_unique<BoneNode>();
        node->index = i;
        node->name = pmxData.bones[i].name;
        node->parentIdx = pmxData.bones[i].parentIdx;
        node->restPosition = pmxData.bones[i].position;
        node->isIK = pmxData.bones[i].isIK;
        node->transAfterPhys = pmxData.bones[i].transAfterPhys;
        node->deformDepth = pmxData.bones[i].deformDepth;
        node->hasAppendRotate = pmxData.bones[i].hasAdditionalRotate;
        node->hasAppendTranslate = pmxData.bones[i].hasAdditionalLocate;
        node->appendBoneIdx = pmxData.bones[i].appendBoneIdx;
        node->appendWeight = pmxData.bones[i].appendWeight;
        node->ikTargetIdx = pmxData.bones[i].ikTargetIdx;
        node->ikLoopCount = pmxData.bones[i].ikLoopCount;
        node->ikRotationLimit = pmxData.bones[i].ikRotationLimit;
        for (const auto& link : pmxData.bones[i].ikLinks) {
            BoneNode::IKLink ikLink;
            ikLink.boneIdx = link.boneIdx;
            ikLink.hasLimit = link.hasLimit;
            ikLink.limitMin = link.limitMin;
            ikLink.limitMax = link.limitMax;
            node->ikLinks.push_back(ikLink);
        }
        m_boneMap[node->name] = node.get();
        m_bones.push_back(std::move(node));
    }
    for (int i = 0; i < n; i++) {
        int p = m_bones[i]->parentIdx;
        if (p >= 0 && p < n) {
            m_bones[p]->children.push_back(i);
        }
    }
    // Convert absolute PMX positions to relative (like saba: child - parent).
    std::vector<btVector3> absPositions(n);
    for (int i = 0; i < n; i++) absPositions[i] = m_bones[i]->restPosition;
    for (int i = 0; i < n; i++) {
        int p = m_bones[i]->parentIdx;
        if (p >= 0 && p < n) {
            m_bones[i]->restPosition = absPositions[i] - absPositions[p];
        }
    }
    // Initialize morph names and weights
    for (const auto& m : pmxData.morphs) {
        m_morphNames.push_back(m.name);
        m_morphWeights.push_back(0.0f);
    }
    return true;
}

BoneNode* Model::getBone(const std::string& name) {
    auto it = m_boneMap.find(name);
    return it != m_boneMap.end() ? it->second : nullptr;
}

const BoneNode* Model::getBone(const std::string& name) const {
    auto it = m_boneMap.find(name);
    return it != m_boneMap.end() ? it->second : nullptr;
}

void Model::resetAnimation() {
    for (auto& b : m_bones) {
        b->animTranslate = btVector3(0,0,0);
        b->animRotate    = btQuaternion(0,0,0,1);
        b->ikTranslate   = btVector3(0,0,0);
        b->ikRotate      = btQuaternion(0,0,0,1);
    }
}

void Model::saveBaseAnimation() {
    for (auto& b : m_bones) {
        b->baseAnimTranslate = b->animTranslate;
        b->baseAnimRotate = b->animRotate;
    }
}

void Model::loadBaseAnimation() {
    for (auto& b : m_bones) {
        b->animTranslate = b->baseAnimTranslate;
        b->animRotate = b->baseAnimRotate;
    }
}

int Model::findMorphIndex(const std::string& name) const {
    for (size_t i = 0; i < m_morphNames.size(); i++) {
        if (m_morphNames[i] == name) return (int)i;
    }
    return -1;
}

float* Model::getMorphWeightPtr(size_t idx) {
    if (idx >= m_morphWeights.size()) return nullptr;
    return &m_morphWeights[idx];
}

} // namespace mmp::model

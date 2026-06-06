#include "model.h"
#include "core/util/logger.h"
#include <algorithm>
#include <functional>

namespace mmbp::model {

bool Model::load(const pmx::PMXData &pmxData) {
  m_bones.clear();
  m_boneMap.clear();
  m_morphWeights.clear();
  m_morphNames.clear();
  m_morphMap.clear();
  int n = static_cast<int>(pmxData.bones.size());
  m_bones.reserve(n);
  for (int i = 0; i < n; i++) {
    auto node = std::make_unique<BoneNode>();
    node->index = i;
    node->name = pmxData.bones[i].name;
    node->parentIdx = pmxData.bones[i].parentIdx;
    node->restPosition = pmxData.bones[i].position;

    node->initialTranslate = pmxData.bones[i].position;
    node->initialRotate = btQuaternion(0, 0, 0, 1);
    node->isIK = pmxData.bones[i].isIK;
    node->transAfterPhys = pmxData.bones[i].transAfterPhys;
    node->deformDepth = pmxData.bones[i].deformDepth;
    node->hasAppendRotate = pmxData.bones[i].hasAdditionalRotate;
    node->hasAppendTranslate = pmxData.bones[i].hasAdditionalLocate;
    node->appendLocal = pmxData.bones[i].appendLocal;
    node->appendBoneIdx = pmxData.bones[i].appendBoneIdx;
    node->appendWeight = pmxData.bones[i].appendWeight;
    node->ikTargetIdx = pmxData.bones[i].ikTargetIdx;
    node->ikLoopCount = pmxData.bones[i].ikLoopCount;
    node->ikRotationLimit = pmxData.bones[i].ikRotationLimit;
    for (const auto &link : pmxData.bones[i].ikLinks) {
      BoneNode::IKLink ikLink;
      ikLink.boneIdx = link.boneIdx;
      ikLink.hasLimit = link.hasLimit;
      ikLink.limitMin = link.limitMin;
      ikLink.limitMax = link.limitMax;
      node->ikLinks.push_back(ikLink);
    }

    auto it = m_boneMap.find(node->name);
    if (it != m_boneMap.end()) {

      g_logger.warn(
          "[MMBP] duplicate bone name '%s' at index %d (overwriting index %d)",
          node->name.c_str(), i, it->second->index);

      m_orphanBoneCount++;
    }
    m_boneMap[node->name] = node.get();
    m_bones.push_back(std::move(node));
  }

  std::vector<bool> visited(n, false);
  for (int i = 0; i < n; i++) {
    int p = m_bones[i]->parentIdx;
    if (p >= 0 && p < n && p != i) {
      m_bones[p]->children.push_back(i);
    }
  }

  {
    std::vector<int> color(n, 0);
    std::function<bool(int)> dfs = [&](int node) -> bool {
      if (color[node] == 2)
        return false;
      if (color[node] == 1)
        return true;
      color[node] = 1;
      int p = m_bones[node]->parentIdx;
      if (p >= 0 && p < n && p != node) {
        if (dfs(p))
          return true;
      }
      color[node] = 2;
      return false;
    };
    for (int i = 0; i < n; i++) {
      if (color[i] == 0 && dfs(i)) {
        g_logger.warn("[MMBP] cycle detected in bone hierarchy at bone %d "
                      "('%s'), breaking link",
                      i, m_bones[i]->name.c_str());

        int oldParent = m_bones[i]->parentIdx;
        if (oldParent >= 0 && oldParent < n) {
          auto &siblings = m_bones[oldParent]->children;
          siblings.erase(std::remove(siblings.begin(), siblings.end(), i),
                         siblings.end());
        }
        m_bones[i]->parentIdx = -1;
      }
    }
  }

  std::vector<btVector3> absPositions(n);
  for (int i = 0; i < n; i++)
    absPositions[i] = m_bones[i]->restPosition;
  for (int i = 0; i < n; i++) {
    int p = m_bones[i]->parentIdx;
    if (p >= 0 && p < n) {
      m_bones[i]->restPosition = absPositions[i] - absPositions[p];
      m_bones[i]->initialTranslate = absPositions[i] - absPositions[p];
    }
  }

  for (size_t i = 0; i < pmxData.morphs.size(); i++) {
    m_morphNames.push_back(pmxData.morphs[i].name);
    m_morphWeights.push_back(0.0f);
    m_morphMap[pmxData.morphs[i].name] = static_cast<int>(i);
  }

  m_topoValid = true;
  for (int i = 0; i < n; i++) {
    auto *bone = m_bones[i].get();
    if (bone && bone->parentIdx >= 0 && bone->parentIdx >= i) {
      m_topoValid = false;
      break;
    }
  }

  if (m_topoValid) {
    m_topologyVersion++;
  } else {
    m_topologyVersion = 0;
  }

  if (m_orphanBoneCount > 0) {
    g_logger.warn("[MMBP] model load: %zu bone(s) orphaned by duplicate names "
                  "(still in m_bones, but unreachable via m_boneMap). "
                  "Animation / IK on those bones will be silently ignored. "
                  "Consider renaming in the source PMX file.",
                  m_orphanBoneCount);
  }
  return true;
}

BoneNode *Model::getBone(const std::string &name) {
  auto it = m_boneMap.find(name);
  return it != m_boneMap.end() ? it->second : nullptr;
}

const BoneNode *Model::getBone(const std::string &name) const {
  auto it = m_boneMap.find(name);
  return it != m_boneMap.end() ? it->second : nullptr;
}

void Model::resetAnimation() {
  for (auto &b : m_bones) {
    b->animTranslate = btVector3(0, 0, 0);
    b->animRotate = btQuaternion(0, 0, 0, 1);
    b->ikTranslate = btVector3(0, 0, 0);
    b->ikRotate = btQuaternion(0, 0, 0, 1);
    b->appendTranslate = btVector3(0, 0, 0);
    b->appendRotate = btQuaternion(0, 0, 0, 1);

    b->baseAnimTranslate = btVector3(0, 0, 0);
    b->baseAnimRotate = btQuaternion(0, 0, 0, 1);

    b->local = btTransform::getIdentity();
    b->global = btTransform::getIdentity();
  }
}

void Model::saveBaseAnimation() {
  for (auto &b : m_bones) {
    b->baseAnimTranslate = b->animTranslate;
    b->baseAnimRotate = b->animRotate;
  }
}

void Model::loadBaseAnimation() {
  for (auto &b : m_bones) {
    b->animTranslate = b->baseAnimTranslate;
    b->animRotate = b->baseAnimRotate;
  }
}

int Model::findMorphIndex(const std::string &name) const {
  auto it = m_morphMap.find(name);
  return it != m_morphMap.end() ? it->second : -1;
}

float *Model::getMorphWeightPtr(size_t idx) {
  if (idx >= m_morphWeights.size())
    return nullptr;
  return &m_morphWeights[idx];
}

} // namespace mmbp::model

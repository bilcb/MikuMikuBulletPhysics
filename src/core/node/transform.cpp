#include "transform.h"
#include "core/util/logger.h"
#include <cstdint>

namespace mmbp::node {

static constexpr int32_t MAX_RECURSION_DEPTH = 256;

void updateLocalTransform(model::BoneNode &bone) {

  btTransform t;
  t.setIdentity();
  btVector3 origin =
      bone.initialTranslate + bone.animTranslate + bone.ikTranslate;
  btQuaternion rot = bone.initialRotate * bone.ikRotate * bone.animRotate;
  if (bone.hasAppendTranslate)
    origin += bone.appendTranslate;
  if (bone.hasAppendRotate)
    rot = rot * bone.appendRotate;
  t.setOrigin(origin);
  t.setRotation(rot);
  bone.local = t;
}

static void updateGlobalTransformImpl(model::BoneNode &bone,
                                      model::Model &model, int32_t depth) {
  if (depth > MAX_RECURSION_DEPTH) {

    g_logger.warn("[MMBP] updateGlobalTransform: recursion depth limit (%d) "
                  "exceeded at bone '%s' — "
                  "possible cycle or extremely deep hierarchy",
                  MAX_RECURSION_DEPTH, bone.name.c_str());
    return;
  }
  if (bone.parentIdx >= 0) {
    auto *parent = model.getBone(bone.parentIdx);
    if (parent)
      bone.global = parent->global * bone.local;
    else
      bone.global = bone.local;
  } else {
    bone.global = bone.local;
  }
  for (int32_t childIdx : bone.children) {
    auto *child = model.getBone(childIdx);
    if (child) {
      updateLocalTransform(*child);
      updateGlobalTransformImpl(*child, model, depth + 1);
    }
  }
}

void updateGlobalTransform(model::BoneNode &bone, model::Model &model) {
  updateGlobalTransformImpl(bone, model, 0);
}

static void updateFilteredGlobalTransformImpl(model::BoneNode &bone,
                                              model::Model &model,
                                              bool afterPhysics,
                                              int32_t depth) {
  if (depth > MAX_RECURSION_DEPTH) {
    g_logger.warn("[MMBP] updateFilteredGlobalTransform: recursion depth limit "
                  "(%d) exceeded at bone '%s'",
                  MAX_RECURSION_DEPTH, bone.name.c_str());
    return;
  }
  if (bone.parentIdx >= 0) {
    auto *parent = model.getBone(bone.parentIdx);
    if (parent)
      bone.global = parent->global * bone.local;
    else
      bone.global = bone.local;
  } else {
    bone.global = bone.local;
  }
  for (int32_t childIdx : bone.children) {
    auto *child = model.getBone(childIdx);
    if (child) {
      if (child->transAfterPhys == afterPhysics) {
        updateLocalTransform(*child);
        updateFilteredGlobalTransformImpl(*child, model, afterPhysics,
                                          depth + 1);
      }
    }
  }
}

void updateFilteredGlobalTransform(model::BoneNode &bone, model::Model &model,
                                   bool afterPhysics) {
  updateFilteredGlobalTransformImpl(bone, model, afterPhysics, 0);
}

static void updateChildTransformsImpl(model::BoneNode &bone,
                                      model::Model &model, int32_t depth) {
  if (depth > MAX_RECURSION_DEPTH) {
    g_logger.warn("[MMBP] updateChildTransforms: recursion depth limit (%d) "
                  "exceeded at bone '%s'",
                  MAX_RECURSION_DEPTH, bone.name.c_str());
    return;
  }
  for (int32_t childIdx : bone.children) {
    auto *child = model.getBone(childIdx);
    if (child) {
      updateLocalTransform(*child);
      child->global = bone.global * child->local;
      updateChildTransformsImpl(*child, model, depth + 1);
    }
  }
}

void updateChildTransforms(model::BoneNode &bone, model::Model &model) {
  updateChildTransformsImpl(bone, model, 0);
}

void updateAllBoneTransforms(model::Model &model) {
  updateAppendTransforms(model);
  size_t n = model.boneCount();

  if (model.isTopoValid()) {

    for (size_t i = 0; i < n; i++) {
      auto *bone = model.getBone(i);
      if (!bone)
        continue;
      updateLocalTransform(*bone);
      if (bone->parentIdx >= 0) {
        auto *parent = model.getBone(bone->parentIdx);
        if (parent)
          bone->global = parent->global * bone->local;
        else
          bone->global = bone->local;
      } else {
        bone->global = bone->local;
      }
    }
  } else {

    for (size_t i = 0; i < n; i++) {
      auto *bone = model.getBone(i);
      if (bone && bone->parentIdx < 0) {
        updateLocalTransform(*bone);
        updateGlobalTransformImpl(*bone, model, 0);
      }
    }
  }

#ifndef NDEBUG
  for (size_t i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (!bone)
      continue;
    const btVector3 &o = bone->global.getOrigin();
    if (!(std::isfinite(o.x()) && std::isfinite(o.y()) && std::isfinite(o.z())))
      g_logger.warn("[MMBP] updateAllBoneTransforms: non-finite global origin "
                    "at bone '%s' (idx=%zu)",
                    bone->name.c_str(), i);
  }
#endif
}

void updateAppendTransforms(model::Model &model) {
  size_t n = model.boneCount();

  for (size_t i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (bone) {
      bone->appendTranslate = btVector3(0, 0, 0);
      bone->appendRotate = btQuaternion(0, 0, 0, 1);
    }
  }

  std::vector<bool> resolved(n, false);

  for (size_t i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (!bone || bone->appendBoneIdx < 0) {
      resolved[i] = true;
      continue;
    }
    if (!bone->appendLocal)
      continue;
    auto *src = model.getBone(bone->appendBoneIdx);
    if (!src) {
      resolved[i] = true;
      continue;
    }

    btVector3 srcTrans = src->animTranslate;
    btQuaternion srcRot = src->ikRotate * src->animRotate;

    if (bone->hasAppendTranslate)
      bone->appendTranslate = srcTrans * bone->appendWeight;
    if (bone->hasAppendRotate)
      bone->appendRotate =
          btQuaternion::getIdentity().slerp(srcRot, bone->appendWeight);
    resolved[i] = true;
  }

  bool progress = true;
  int maxIterations = static_cast<int>(n) + 1;
  while (progress && maxIterations-- > 0) {
    progress = false;
    for (size_t i = 0; i < n; i++) {
      if (resolved[i])
        continue;
      auto *bone = model.getBone(i);
      if (!bone || bone->appendBoneIdx < 0) {
        resolved[i] = true;
        continue;
      }
      auto *src = model.getBone(bone->appendBoneIdx);
      if (!src) {
        resolved[i] = true;
        continue;
      }
      size_t srcIdx = static_cast<size_t>(bone->appendBoneIdx);
      if (!resolved[srcIdx])
        continue;

      btVector3 srcTrans;
      btQuaternion srcRot;

      if (src->appendBoneIdx >= 0 &&
          (src->hasAppendTranslate || src->hasAppendRotate)) {
        srcTrans = src->appendTranslate;
        srcRot = src->appendRotate;
      } else {
        srcTrans = src->animTranslate;
        srcRot = src->ikRotate * src->animRotate;
      }
      if (bone->hasAppendTranslate)
        bone->appendTranslate = srcTrans * bone->appendWeight;
      if (bone->hasAppendRotate)
        bone->appendRotate =
            btQuaternion::getIdentity().slerp(srcRot, bone->appendWeight);
      resolved[i] = true;
      progress = true;
    }
  }

  if (maxIterations <= 0) {
    g_logger.warn("[MMBP] Append transform resolution reached iteration limit "
                  "— unresolved bones may exist");
  }
}

} // namespace mmbp::node

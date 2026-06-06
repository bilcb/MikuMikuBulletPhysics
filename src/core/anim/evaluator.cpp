#include "evaluator.h"
#include <algorithm>
#include <cmath>

namespace mmbp::anim {

struct VMDBezier {
  float cp1x, cp1y, cp2x, cp2y;
  VMDBezier() : cp1x(0), cp1y(0), cp2x(1), cp2y(1) {}
  void set(const uint8_t *cp) {
    cp1x = cp[0] / 127.0f;
    cp1y = cp[4] / 127.0f;
    cp2x = cp[8] / 127.0f;
    cp2y = cp[12] / 127.0f;
  }
  float evalX(float t) const {
    float t2 = t * t, t3 = t2 * t, u = 1.0f - t, u2 = u * u;
    return t3 + 3.0f * t2 * u * cp2x + 3.0f * t * u2 * cp1x;
  }
  float evalY(float t) const {
    float t2 = t * t, t3 = t2 * t, u = 1.0f - t, u2 = u * u;
    return t3 + 3.0f * t2 * u * cp2y + 3.0f * t * u2 * cp1y;
  }

  float findX(float time, float initialGuess = -1.0f) const {
    float t = (initialGuess >= 0.0f) ? initialGuess : time;
    bool newtonConverged = false;
    for (int i = 0; i < 4; i++) {
      float x = evalX(t);
      float err = x - time;
      if (std::abs(err) < 0.001f) {
        newtonConverged = true;
        break;
      }
      float u = 1.0f - t;
      float dx = 3.0f * u * u * cp1x + 6.0f * u * t * (cp2x - cp1x) +
                 3.0f * t * t * (1.0f - cp2x);
      if (std::abs(dx) < 1e-6f)
        break;
      t -= err / dx;
      t = std::max(0.0f, std::min(1.0f, t));
    }
    if (newtonConverged)
      return t;

    float lo = 0.0f, hi = 1.0f;
    for (int i = 0; i < 24; i++) {
      float mid = 0.5f * (lo + hi);
      float x = evalX(mid);
      if (std::abs(x - time) < 0.001f)
        return mid;
      if (x < time)
        lo = mid;
      else
        hi = mid;
    }
    return 0.5f * (lo + hi);
  }
};

template <typename KeyType>
static size_t findBoundKey(const std::vector<KeyType *> &keys, uint32_t frame,
                           size_t startIdx) {
  if (keys.empty() || startIdx >= keys.size())
    return keys.size();

  if (startIdx < keys.size() && keys[startIdx]->frame <= frame) {
    if (startIdx + 1 < keys.size() && keys[startIdx + 1]->frame > frame) {
      return startIdx + 1;
    }
  } else if (startIdx > 0 && keys[startIdx - 1]->frame <= frame) {
    if (startIdx < keys.size() && keys[startIdx]->frame > frame) {
      return startIdx;
    }
  }

  auto it = std::upper_bound(
      keys.begin(), keys.end(), frame,
      [](uint32_t f, const KeyType *kf) { return f < kf->frame; });
  return std::distance(keys.begin(), it);
}

void BoneController::evaluate(model::BoneNode *bone, float frame,
                              float weight) {
  if (keys.empty())
    return;
  if (frame < 0.0f)
    frame = 0.0f;

  size_t idx = findBoundKey(keys, (uint32_t)frame, startKeyIndex);
  if (idx > 0 && idx <= keys.size()) {
    startKeyIndex = idx - 1;
  }

  const vmd::VMDKeyframe *k0 = nullptr;
  const vmd::VMDKeyframe *k1 = nullptr;

  if (idx == 0) {
    k1 = keys[0];
  } else if (idx >= keys.size()) {
    k0 = keys[keys.size() - 1];
  } else {
    k0 = keys[idx - 1];
    k1 = keys[idx];
  }

  if (!k0 && !k1)
    return;
  if (!k1 || k0 == k1) {
    if (weight == 1.0f) {
      bone->animTranslate = k0->translation;
      bone->animRotate = k0->rotation;
    } else {
      bone->animTranslate = bone->animTranslate +
                            (k0->translation - bone->animTranslate) * weight;
      bone->animRotate = bone->animRotate.slerp(k0->rotation, weight);
    }
    return;
  }
  if (!k0) {
    if (weight == 1.0f) {
      bone->animTranslate = k1->translation;
      bone->animRotate = k1->rotation;
    } else {
      bone->animTranslate = bone->animTranslate +
                            (k1->translation - bone->animTranslate) * weight;
      bone->animRotate = bone->animRotate.slerp(k1->rotation, weight);
    }
    return;
  }

  float t = (frame - k0->frame) / (float)(k1->frame - k0->frame);
  t = std::max(0.0f, std::min(1.0f, t));

  VMDBezier bx, by, bz, br;
  bx.set(k0->interpX);
  by.set(k0->interpY);
  bz.set(k0->interpZ);
  br.set(k0->interpRot);

  bool continuous =
      (std::abs(frame - lastFrame - 1.0f) < 0.5f && lastFrame > 0);
  float tx = bx.evalY(bx.findX(t, continuous ? lastTx : -1.0f));
  float ty = by.evalY(by.findX(t, continuous ? lastTy : -1.0f));
  float tz = bz.evalY(bz.findX(t, continuous ? lastTz : -1.0f));
  float tr = br.evalY(br.findX(t, continuous ? lastTr : -1.0f));
  lastFrame = frame;
  lastTx = tx;
  lastTy = ty;
  lastTz = tz;
  lastTr = tr;

  btVector3 targetTranslate(
      k0->translation.x() + (k1->translation.x() - k0->translation.x()) * tx,
      k0->translation.y() + (k1->translation.y() - k0->translation.y()) * ty,
      k0->translation.z() + (k1->translation.z() - k0->translation.z()) * tz);
  btQuaternion targetRotate = k0->rotation.slerp(k1->rotation, tr);

  if (weight == 1.0f) {
    bone->animTranslate = targetTranslate;
    bone->animRotate = targetRotate;
  } else {
    bone->animTranslate =
        bone->animTranslate + (targetTranslate - bone->animTranslate) * weight;
    bone->animRotate = bone->animRotate.slerp(targetRotate, weight);
  }
}

void MorphController::evaluate(float *weightPtr, float frame, float weight) {
  if (keys.empty() || !weightPtr)
    return;
  if (frame < 0.0f)
    frame = 0.0f;

  size_t idx = findBoundKey(keys, (uint32_t)frame, startKeyIndex);
  if (idx > 0 && idx <= keys.size()) {
    startKeyIndex = idx - 1;
  }

  const vmd::MorphKeyframe *k0 = nullptr;
  const vmd::MorphKeyframe *k1 = nullptr;

  if (idx == 0) {
    k1 = keys[0];
  } else if (idx >= keys.size()) {
    k0 = keys[keys.size() - 1];
  } else {
    k0 = keys[idx - 1];
    k1 = keys[idx];
  }

  if (!k0 && !k1)
    return;
  float targetWeight;
  if (!k1) {
    targetWeight = k0->weight;
  } else if (!k0) {
    targetWeight = k1->weight;
  } else {
    float t = (frame - k0->frame) / (float)(k1->frame - k0->frame);
    t = std::max(0.0f, std::min(1.0f, t));
    targetWeight = k0->weight + (k1->weight - k0->weight) * t;
  }

  if (weight == 1.0f) {
    *weightPtr = targetWeight;
  } else {
    *weightPtr = *weightPtr + (targetWeight - *weightPtr) * weight;
  }

  if (*weightPtr < 0.0f)
    *weightPtr = 0.0f;
  if (*weightPtr > 1.0f)
    *weightPtr = 1.0f;
}

void evaluate(AnimationState &state, const vmd::VMDData &vmd,
              model::Model &model, float frame, float weight) {

  if (!state.initialized) {
    state.boneControllers.clear();
    if (!vmd.boneKeyframeGroups.empty()) {

      for (const auto &[name, indices] : vmd.boneKeyframeGroups) {
        auto &ctrl = state.boneControllers[name];
        ctrl.keys.reserve(indices.size());
        for (size_t idx : indices) {
          ctrl.keys.push_back(&vmd.keyframes[idx]);
        }
        ctrl.startKeyIndex = 0;
      }
    } else {

      for (const auto &kf : vmd.keyframes) {
        state.boneControllers[kf.boneName].keys.push_back(&kf);
      }
      for (auto &[name, ctrl] : state.boneControllers) {
        std::sort(ctrl.keys.begin(), ctrl.keys.end(),
                  [](const vmd::VMDKeyframe *a, const vmd::VMDKeyframe *b) {
                    return a->frame < b->frame;
                  });

        std::reverse(ctrl.keys.begin(), ctrl.keys.end());
        auto last = std::unique(
            ctrl.keys.begin(), ctrl.keys.end(),
            [](const vmd::VMDKeyframe *a, const vmd::VMDKeyframe *b) {
              return a->frame == b->frame;
            });
        ctrl.keys.erase(last, ctrl.keys.end());
        std::reverse(ctrl.keys.begin(), ctrl.keys.end());
        ctrl.startKeyIndex = 0;
      }
    }
    state.initialized = true;
  }

  for (auto &[name, ctrl] : state.boneControllers) {
    auto *bone = model.getBone(name);
    if (!bone)
      continue;
    ctrl.evaluate(bone, frame, weight);
  }
}

void evaluateMorphs(AnimationState &state, const vmd::VMDData &vmd,
                    model::Model &model, float frame, float weight) {

  if (!state.morphInitialized) {
    state.morphControllers.clear();
    if (!vmd.morphKeyframeGroups.empty()) {

      for (const auto &[name, indices] : vmd.morphKeyframeGroups) {
        auto &ctrl = state.morphControllers[name];
        ctrl.keys.reserve(indices.size());
        for (size_t idx : indices) {
          ctrl.keys.push_back(&vmd.morphKeys[idx]);
        }
        ctrl.startKeyIndex = 0;
      }
    } else {

      for (const auto &mk : vmd.morphKeys) {
        state.morphControllers[mk.name].keys.push_back(&mk);
      }
      for (auto &[name, ctrl] : state.morphControllers) {
        std::sort(ctrl.keys.begin(), ctrl.keys.end(),
                  [](const vmd::MorphKeyframe *a, const vmd::MorphKeyframe *b) {
                    return a->frame < b->frame;
                  });
        ctrl.startKeyIndex = 0;
      }
    }
    state.morphInitialized = true;
  }

  for (auto &[name, ctrl] : state.morphControllers) {
    int idx = model.findMorphIndex(name);
    if (idx < 0)
      continue;
    float *wp = model.getMorphWeightPtr(idx);
    ctrl.evaluate(wp, frame, weight);
  }
}

AnimationState::~AnimationState() = default;

void AnimationState::reset() {
  boneControllers.clear();
  morphControllers.clear();
  vmdVersion = 0;
  initialized = false;
  morphInitialized = false;
  ikKeyCursor = 0;
  lastIKFrame = -1.0f;
}

void AnimationState::invalidate() {

  vmdVersion++;
  initialized = false;
  morphInitialized = false;
  morphControllers.clear();
  ikKeyCursor = 0;
  lastIKFrame = -1.0f;
}

void evaluateIKKeys(AnimationState &state, const vmd::VMDData &vmd,
                    model::Model &model, float frame) {
  if (vmd.ikKeys.empty())
    return;
  if (frame < 0.0f)
    frame = 0.0f;

  if (frame < state.lastIKFrame) {
    state.ikKeyCursor = 0;
  }
  state.lastIKFrame = frame;

  for (size_t i = state.ikKeyCursor; i < vmd.ikKeys.size(); i++) {
    const auto &ik = vmd.ikKeys[i];
    if (ik.frame > (uint32_t)frame)
      break;
    for (const auto &st : ik.states) {
      auto *bone = model.getBone(st.name);
      if (bone && bone->isIK)
        bone->ikEnabled = st.enabled;
    }
    state.ikKeyCursor = i + 1;
  }
}

void groupKeyframesByBone(vmd::VMDData &vmd) {
  vmd.boneKeyframeGroups.clear();

  vmd.boneKeyframeGroups.reserve(vmd.keyframes.size());
  for (size_t i = 0; i < vmd.keyframes.size(); i++) {
    vmd.boneKeyframeGroups[vmd.keyframes[i].boneName].push_back(i);
  }
  for (auto &[name, indices] : vmd.boneKeyframeGroups) {
    std::sort(indices.begin(), indices.end(), [&vmd](size_t a, size_t b) {
      return vmd.keyframes[a].frame < vmd.keyframes[b].frame;
    });
  }
}

void groupMorphKeyframes(vmd::VMDData &vmd) {
  vmd.morphKeyframeGroups.clear();
  vmd.morphKeyframeGroups.reserve(vmd.morphKeys.size());
  for (size_t i = 0; i < vmd.morphKeys.size(); i++) {
    vmd.morphKeyframeGroups[vmd.morphKeys[i].name].push_back(i);
  }
  for (auto &[name, indices] : vmd.morphKeyframeGroups) {
    std::sort(indices.begin(), indices.end(), [&vmd](size_t a, size_t b) {
      return vmd.morphKeys[a].frame < vmd.morphKeys[b].frame;
    });
  }
}

} // namespace mmbp::anim

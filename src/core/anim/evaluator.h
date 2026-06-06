#pragma once
#include "core/model/model.h"
#include "core/vmd/types.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace mmbp::anim {

struct BoneController {
  std::vector<const vmd::VMDKeyframe *> keys;
  size_t startKeyIndex = 0;
  float lastFrame = -1.0f;
  float lastTx = 0.0f;
  float lastTy = 0.0f;
  float lastTz = 0.0f;
  float lastTr = 0.0f;
  void evaluate(model::BoneNode *bone, float frame, float weight = 1.0f);
};

struct MorphController {
  std::vector<const vmd::MorphKeyframe *> keys;
  size_t startKeyIndex = 0;
  void evaluate(float *weightPtr, float frame, float weight = 1.0f);
};

struct AnimationState {
  std::unordered_map<std::string, BoneController> boneControllers;
  std::unordered_map<std::string, MorphController> morphControllers;
  size_t vmdVersion = 0;
  bool initialized = false;
  bool morphInitialized = false;
  size_t ikKeyCursor = 0;
  float lastIKFrame = -1.0f;

  ~AnimationState();
  void reset();
  void invalidate();
};

void evaluate(AnimationState &state, const vmd::VMDData &vmd,
              model::Model &model, float frame, float weight = 1.0f);
void evaluateMorphs(AnimationState &state, const vmd::VMDData &vmd,
                    model::Model &model, float frame, float weight = 1.0f);
void evaluateIKKeys(AnimationState &state, const vmd::VMDData &vmd,
                    model::Model &model, float frame);

void groupKeyframesByBone(vmd::VMDData &vmd);
void groupMorphKeyframes(vmd::VMDData &vmd);

} // namespace mmbp::anim

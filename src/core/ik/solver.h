#pragma once
#include "core/model/model.h"

namespace mmbp::ik {

struct IkConfig {
  float distDegradeRatio = 2.0f;

  float maxStepAngle = SIMD_PI * 0.25f;

  bool enableLimitRemapping = false;
};

const IkConfig *ikConfig();

void solve(model::Model &model, const model::BoneNode &ikBone);

} // namespace mmbp::ik

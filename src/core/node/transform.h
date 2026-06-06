#pragma once
#include "core/model/model.h"

namespace mmbp::node {

void updateLocalTransform(model::BoneNode &bone);
void updateGlobalTransform(model::BoneNode &bone, model::Model &model);
void updateFilteredGlobalTransform(model::BoneNode &bone, model::Model &model,
                                   bool afterPhysics);
void updateChildTransforms(model::BoneNode &bone, model::Model &model);
void updateAppendTransforms(model::Model &model);
void updateAllBoneTransforms(model::Model &model);

} // namespace mmbp::node

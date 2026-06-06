#include "PhysicsPipeline.h"
#include "../adapter/BlenderAdapter.h"
#include "core/anim/evaluator.h"
#include "core/ik/solver.h"
#include "core/model/model.h"
#include "core/node/transform.h"
#include "core/physics/world.h"
#include "core/vmd/types.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace mmbp::bridge {

static void updateAllBoneTransformsFiltered(model::Model &model,
                                            bool afterPhysics) {
  size_t n = model.boneCount();
  for (size_t i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (bone && bone->transAfterPhys == afterPhysics)
      node::updateLocalTransform(*bone);
  }
  for (size_t i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (bone && bone->parentIdx < 0 && bone->transAfterPhys == afterPhysics) {
      if (afterPhysics) {
        node::updateFilteredGlobalTransform(*bone, model, afterPhysics);
      } else {
        node::updateGlobalTransform(*bone, model);
        node::updateChildTransforms(*bone, model);
      }
    }
  }
}

static void runIKForBones(model::Model &model, bool afterPhysics) {
  for (size_t i = 0; i < model.boneCount(); i++) {
    auto *bone = model.getBone(i);
    if (bone && bone->isIK && bone->ikEnabled &&
        bone->transAfterPhys == afterPhysics) {
      ik::solve(model, *bone);
    }
  }
}

void PhysicsPipeline::animate(anim::AnimationState &animState,
                              model::Model &model, const vmd::VMDData &vmdData,
                              float frame) {
  model.resetAnimation();
  anim::evaluate(animState, vmdData, model, frame);
  anim::evaluateIKKeys(animState, vmdData, model, frame);

  anim::evaluateMorphs(animState, vmdData, model, frame);
}

void PhysicsPipeline::prePhysics(model::Model &model) {
  updateAllBoneTransformsFiltered(model, false);
  runIKForBones(model, false);
  node::updateAllBoneTransforms(model);
}

void PhysicsPipeline::simulate(physics::World *world, model::Model &model,
                               float delta) {
  if (!world)
    return;

  world->step(delta);
  world->syncBoneTransforms(model);
}

void PhysicsPipeline::postPhysics(model::Model &model) {
  updateAllBoneTransformsFiltered(model, true);
  runIKForBones(model, true);
}

static bool convertToParentRelative(const model::Model &model,
                                    const model::BoneNode &bone, int i, int n,
                                    float scale, btVector3 &pos,
                                    btQuaternion &rot) {
  if (bone.parentIdx < 0 || bone.parentIdx >= n)
    return false;
  auto *parent = model.getBone(bone.parentIdx);
  if (!parent)
    return false;
  btVector3 pPos = parent->global.getOrigin() * scale;
  btQuaternion parentRotInv = parent->global.getRotation().inverse();
  pos = quatRotate(parentRotInv, pos - pPos);
  rot = parentRotInv * rot;
  return true;
}

static void applyRestMatrix(const btMatrix3x3 &rm, btVector3 &pos,
                            btQuaternion &rot) {
  pos = rm * pos;
  btMatrix3x3 rMat;
  rMat.setRotation(rot);
  rMat = rm * rMat * rm.transpose();
  rMat.getRotation(rot);
}

static void writeBoneOutput(const btVector3 &pos, const btQuaternion &rot,
                            float *outLocs, float *outQuats, int i) {
  outLocs[i * 3 + 0] = std::isfinite(pos.x()) ? pos.x() : 0.0f;
  outLocs[i * 3 + 1] = std::isfinite(pos.y()) ? pos.y() : 0.0f;
  outLocs[i * 3 + 2] = std::isfinite(pos.z()) ? pos.z() : 0.0f;
  outQuats[i * 4 + 0] = std::isfinite(rot.w()) ? rot.w() : 1.0f;
  outQuats[i * 4 + 1] = std::isfinite(rot.x()) ? rot.x() : 0.0f;
  outQuats[i * 4 + 2] = std::isfinite(rot.y()) ? rot.y() : 0.0f;
  outQuats[i * 4 + 3] = std::isfinite(rot.z()) ? rot.z() : 0.0f;
}

void PhysicsPipeline::output(model::Model &model,
                             const std::vector<btMatrix3x3> &restMatrices,
                             float scale, float *outLocs, float *outQuats,
                             int boneCount) {
  int n = std::min(boneCount, (int)model.boneCount());

  for (int i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (!bone)
      continue;

    btVector3 pos = bone->global.getOrigin() * scale;
    btQuaternion rot = bone->global.getRotation();

    (void)convertToParentRelative(model, *bone, i, n, scale, pos, rot);

    pos = pos - bone->restPosition * scale;

    if (i < (int)restMatrices.size()) {
      applyRestMatrix(restMatrices[i], pos, rot);
    }

    writeBoneOutput(pos, rot, outLocs, outQuats, i);
  }
}

PhysicsPipeline::StepResult
PhysicsPipeline::step(anim::AnimationState &animState, model::Model &model,
                      physics::World *world, const vmd::VMDData &vmdData,
                      float frame, float delta,
                      const std::vector<btMatrix3x3> &restMatrices, float scale,
                      float *outLocs, float *outQuats, int boneCount) {
  animate(animState, model, vmdData, frame);
  node::updateAppendTransforms(model);
  prePhysics(model);
  simulate(world, model, delta);

  if (m_hasPostPhysicsBones) {
    postPhysics(model);
  }
  output(model, restMatrices, scale, outLocs, outQuats, boneCount);
  return {std::min(boneCount, (int)model.boneCount())};
}

void PhysicsPipeline::updatePostPhysicsFlag(const model::Model &model) {
  m_hasPostPhysicsBones = false;
  for (size_t i = 0; i < model.boneCount(); i++) {
    auto *bone = model.getBone(i);
    if (bone && bone->transAfterPhys) {
      m_hasPostPhysicsBones = true;
      break;
    }
  }
}

PhysicsPipeline::StepResult PhysicsPipeline::resetToAnimation(
    anim::AnimationState &animState, model::Model &model, physics::World *world,
    const vmd::VMDData &vmdData, float frame,
    const std::vector<btMatrix3x3> &restMatrices, float scale, float *outLocs,
    float *outQuats, int boneCount) {

  animate(animState, model, vmdData, frame);

  prePhysics(model);

  if (world) {
    world->resetPhysics(model);
  }

  output(model, restMatrices, scale, outLocs, outQuats, boneCount);
  return {std::min(boneCount, (int)model.boneCount())};
}

PhysicsPipeline::StepResult
PhysicsPipeline::evalOnly(anim::AnimationState &animState, model::Model &model,
                          const vmd::VMDData &vmdData, float frame,
                          const std::vector<btMatrix3x3> &restMatrices,
                          float scale, float *outLocs, float *outQuats,
                          int boneCount) {
  animate(animState, model, vmdData, frame);
  prePhysics(model);
  output(model, restMatrices, scale, outLocs, outQuats, boneCount);
  return {std::min(boneCount, (int)model.boneCount())};
}

} // namespace mmbp::bridge

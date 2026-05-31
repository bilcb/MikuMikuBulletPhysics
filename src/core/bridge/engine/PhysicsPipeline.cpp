#include "PhysicsPipeline.h"
#include "core/model/model.h"
#include "core/physics/world.h"
#include "core/vmd/types.h"
#include "core/anim/evaluator.h"
#include "core/ik/solver.h"
#include "core/node/transform.h"
#include "../adapter/BlenderAdapter.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace mmp::bridge {

static void updateAllBoneTransformsFiltered(model::Model& model, bool afterPhysics) {
    size_t n = model.boneCount();
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        if (!bone || bone->appendBoneIdx < 0) continue;
        if (bone->transAfterPhys != afterPhysics) continue;
        auto* src = model.getBone(bone->appendBoneIdx);
        if (!src) continue;
        btVector3 srcTrans = src->animTranslate;
        btQuaternion srcRot = src->ikRotate * src->animRotate;
        if (bone->hasAppendTranslate)
            bone->appendTranslate = srcTrans * bone->appendWeight;
        if (bone->hasAppendRotate)
            bone->appendRotate = btQuaternion::getIdentity().slerp(srcRot, bone->appendWeight);
    }
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        if (bone && bone->transAfterPhys == afterPhysics)
            node::updateLocalTransform(*bone);
    }
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
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

static void updateAllBoneTransforms(model::Model& model) {
    node::updateAppendTransforms(model);
    size_t n = model.boneCount();
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        if (bone) node::updateLocalTransform(*bone);
    }
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        if (bone && bone->parentIdx < 0) {
            node::updateGlobalTransform(*bone, model);
            node::updateChildTransforms(*bone, model);
        }
    }
}

static void runIKForBones(model::Model& model, bool afterPhysics) {
    for (size_t i = 0; i < model.boneCount(); i++) {
        auto* bone = model.getBone(i);
        if (bone && bone->isIK && bone->ikEnabled && bone->transAfterPhys == afterPhysics) {
            ik::solve(model, *bone);
        }
    }
}

void PhysicsPipeline::animate(model::Model& model, const vmd::VMDData& vmdData, float frame) {
    model.resetAnimation();
    anim::evaluate(vmdData, model, frame);
    anim::evaluateIKKeys(vmdData, model, frame);
    anim::evaluateMorphs(vmdData, model, frame);
}

void PhysicsPipeline::convertVmdToBlenderSpace(model::Model& model) {
    // mmd_tools BoneConverter converts VMD data from MMD space to Blender space:
    //   location: (mat @ location) * scale  where mat = bone.matrix_local.T with rows 1,2 swapped
    //   rotation: q @ rot @ q.conj()  where q = mat.to_quaternion()
    //
    // Our rest matrices encode this conversion (computed by computeBoneRestMatrix).
    // We apply the same conversion to VMD animation data after evaluation.

    for (size_t i = 0; i < model.boneCount(); i++) {
        auto* bone = model.getBone(i);
        if (!bone) continue;

        // Skip bones with no VMD animation
        if (bone->animTranslate.length2() < 1e-10f &&
            bone->animRotate == btQuaternion(0, 0, 0, 1)) {
            continue;
        }

        // Convert VMD rotation using similarity transform (matches mmd_tools BoneConverter)
        // rot_blender = q_rest * rot_mmd * q_rest.conjugated()
        // For now, use the global coordinate conversion as approximation
        // since we don't have per-bone rest matrix rotation at this point
        static const btQuaternion kMmdToBlenderRot(
            btVector3(1, 0, 0), SIMD_PI * 0.5f);
        bone->animRotate = kMmdToBlenderRot * bone->animRotate * kMmdToBlenderRot.inverse();

        // Convert VMD translation: Y↔Z swap (MMD Y-up → Blender Z-up)
        btVector3 t = bone->animTranslate;
        bone->animTranslate = btVector3(t.x(), t.z(), t.y());
    }
}

void PhysicsPipeline::prePhysics(model::Model& model) {
    updateAllBoneTransformsFiltered(model, false);
    runIKForBones(model, false);
    updateAllBoneTransforms(model);
}

void PhysicsPipeline::simulate(physics::World* world, model::Model& model, float delta) {
    if (!world) return;

    for (int i = 0; i < world->rigidBodyCount(); i++) {
        world->setActivation(i, true);
    }

    world->step(delta);
    world->syncBoneTransforms(model);

    for (int i = 0; i < world->rigidBodyCount(); i++) {
        auto* rb = world->getRigidBody(i);
        if (!rb || rb->boneIdx < 0) continue;
        world->cascadeChildTransforms(model, rb->boneIdx);
    }
}

void PhysicsPipeline::postPhysics(model::Model& model) {
    updateAllBoneTransformsFiltered(model, true);
    runIKForBones(model, true);
}

void PhysicsPipeline::output(
    model::Model& model,
    const std::vector<btMatrix3x3>& restMatrices,
    const std::vector<btVector3>& restPosOffsets,
    float scale,
    float* outLocs,
    float* outQuats,
    int boneCount
) {
    int n = std::min(boneCount, (int)model.boneCount());

    for (int i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        btVector3 pos = bone->global.getOrigin();
        btQuaternion rot = bone->global.getRotation();

        pos *= scale;

        // Compute parent-relative local in MMD space
        if (bone->parentIdx >= 0 && bone->parentIdx < n) {
            auto* parent = model.getBone(bone->parentIdx);
            btVector3 pPos = parent->global.getOrigin() * scale;
            btQuaternion pRot = parent->global.getRotation();
            btQuaternion pRotInv = pRot.inverse();
            pos = quatRotate(pRotInv, pos - pPos);
            rot = pRotInv * rot;
        }

        // Subtract rest position to get delta (animation / physics displacement)
        // restPosition is stored in raw MMD units, scale to match
        pos = pos - bone->restPosition * scale;
        // Rotation rest is identity, so delta = local rotation as-is

        // Convert delta from MMD bone-local to Blender bone-local via rest matrix
        if (i < (int)restMatrices.size()) {
            const auto& rm = restMatrices[i];
            pos = rm * pos;
            btMatrix3x3 rMat;
            rMat.setRotation(rot);
            rMat = rm * rMat * rm.transpose();
            rMat.getRotation(rot);
        }

        outLocs[i * 3 + 0] = std::isfinite(pos.x()) ? pos.x() : 0.0f;
        outLocs[i * 3 + 1] = std::isfinite(pos.y()) ? pos.y() : 0.0f;
        outLocs[i * 3 + 2] = std::isfinite(pos.z()) ? pos.z() : 0.0f;
        outQuats[i * 4 + 0] = std::isfinite(rot.w()) ? rot.w() : 1.0f;
        outQuats[i * 4 + 1] = std::isfinite(rot.x()) ? rot.x() : 0.0f;
        outQuats[i * 4 + 2] = std::isfinite(rot.y()) ? rot.y() : 0.0f;
        outQuats[i * 4 + 3] = std::isfinite(rot.z()) ? rot.z() : 0.0f;
    }
}

PhysicsPipeline::StepResult PhysicsPipeline::step(
    model::Model& model,
    physics::World* world,
    const vmd::VMDData& vmdData,
    float frame,
    float delta,
    const std::vector<btMatrix3x3>& restMatrices,
    const std::vector<btVector3>& restPosOffsets,
    float scale,
    float* outLocs,
    float* outQuats,
    int boneCount
) {
    animate(model, vmdData, frame);
    prePhysics(model);
    simulate(world, model, delta);
    postPhysics(model);
    output(model, restMatrices, restPosOffsets, scale, outLocs, outQuats, boneCount);
    return { std::min(boneCount, (int)model.boneCount()) };
}

} // namespace mmp::bridge

#include "solver.h"
#include "core/node/transform.h"
#include "core/math/converter.h"
#include <cmath>
#include <algorithm>

namespace mmp::ik {

static float clampAngle(float angle, float minAngle, float maxAngle) {
    if (minAngle == maxAngle) return minAngle;

    // Normalize to [0, 2pi)
    while (angle >= SIMD_PI * 2.0f) angle -= SIMD_PI * 2.0f;
    while (angle < 0) angle += SIMD_PI * 2.0f;

    // Normalize min/max
    float lo = minAngle;
    float hi = maxAngle;
    while (lo >= SIMD_PI * 2.0f) lo -= SIMD_PI * 2.0f;
    while (lo < 0) lo += SIMD_PI * 2.0f;
    while (hi >= SIMD_PI * 2.0f) hi -= SIMD_PI * 2.0f;
    while (hi < 0) hi += SIMD_PI * 2.0f;

    if (lo <= hi) {
        if (angle >= lo && angle <= hi) return angle;
    } else {
        // Range wraps around
        if (angle >= lo || angle <= hi) return angle;
    }

    // Clamp to nearest boundary
    float diffLo = std::abs(angle - lo);
    float diffHi = std::abs(angle - hi);
    if (diffLo < 2.0f * SIMD_PI - diffLo) diffLo = diffLo; else diffLo = 2.0f * SIMD_PI - diffLo;
    if (diffHi < 2.0f * SIMD_PI - diffHi) diffHi = diffHi; else diffHi = 2.0f * SIMD_PI - diffHi;
    return (diffLo < diffHi) ? lo : hi;
}

// SolvePlane: single-axis constraint solver (knee-like)
// Matches saba's SolvePlane function with accumulated angle and first-iteration inversion check
enum class SolveAxis { X, Y, Z };

static void solvePlane(model::Model& model, const model::BoneNode& ikBone,
                       size_t linkIdx, float limitAngle, SolveAxis axis,
                       int iteration, float& planeModeAngle) {
    const auto& link = ikBone.ikLinks[linkIdx];
    auto* linkBone = model.getBone(link.boneIdx);
    auto* ikBoneMutable = model.getBone(ikBone.index);
    auto* targetBone = model.getBone(ikBone.ikTargetIdx);
    if (!linkBone || !ikBoneMutable || !targetBone) return;

    int rotAxisIdx = 0;
    btVector3 rotAxis(1, 0, 0);

    switch (axis) {
    case SolveAxis::X: rotAxisIdx = 0; rotAxis = btVector3(1, 0, 0); break;
    case SolveAxis::Y: rotAxisIdx = 1; rotAxis = btVector3(0, 1, 0); break;
    case SolveAxis::Z: rotAxisIdx = 2; rotAxis = btVector3(0, 0, 1); break;
    }

    btVector3 ikPos = ikBoneMutable->global.getOrigin();
    btVector3 targetPos = targetBone->global.getOrigin();

    // Transform to chain bone's local space (matches saba)
    btTransform invChain = linkBone->global.inverse();
    btVector3 chainIkPos = invChain * ikPos;
    btVector3 chainTargetPos = invChain * targetPos;

    if (chainIkPos.length2() < 1e-12f || chainTargetPos.length2() < 1e-12f) return;

    btVector3 chainIkVec = chainIkPos.normalized();
    btVector3 chainTargetVec = chainTargetPos.normalized();

    float dot = chainTargetVec.dot(chainIkVec);
    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angle = std::acos(dot);
    angle = std::max(-limitAngle, std::min(limitAngle, angle));

    // Try both directions
    btQuaternion rot1(btVector3(0, 0, 0), 0);
    rot1.setRotation(rotAxis, angle);
    btVector3 targetVec1 = quatRotate(rot1, chainTargetVec);
    float dot1 = targetVec1.dot(chainIkVec);

    btQuaternion rot2(btVector3(0, 0, 0), 0);
    rot2.setRotation(rotAxis, -angle);
    btVector3 targetVec2 = quatRotate(rot2, chainTargetVec);
    float dot2 = targetVec2.dot(chainIkVec);

    // Accumulate angle (matches saba's planeModeAngle tracking)
    float newAngle = planeModeAngle;
    if (dot1 > dot2) {
        newAngle += angle;
    } else {
        newAngle -= angle;
    }

    // First iteration: check if angle needs inversion (matches saba's SolvePlane)
    if (iteration == 0) {
        if (newAngle < link.limitMin[rotAxisIdx] || newAngle > link.limitMax[rotAxisIdx]) {
            if (-newAngle > link.limitMin[rotAxisIdx] && -newAngle < link.limitMax[rotAxisIdx]) {
                newAngle *= -1;
            } else {
                float halfRad = (link.limitMin[rotAxisIdx] + link.limitMax[rotAxisIdx]) * 0.5f;
                if (std::abs(halfRad - newAngle) > std::abs(halfRad + newAngle)) {
                    newAngle *= -1;
                }
            }
        }
    }

    newAngle = std::max(link.limitMin[rotAxisIdx], std::min(link.limitMax[rotAxisIdx], newAngle));
    planeModeAngle = newAngle;

    // Set absolute rotation: ikRot = rotate(newAngle) * inverse(animRotate)
    // This matches saba: ikRotM = rotate(newAngle, axis) * inverse(AnimateRotate)
    btQuaternion absRot(rotAxis, newAngle);
    linkBone->ikRotate = absRot * linkBone->animRotate.inverse();

    node::updateLocalTransform(*linkBone);
    node::updateGlobalTransform(*linkBone, model);
    node::updateChildTransforms(*linkBone, model);
}

void solve(model::Model& model, const model::BoneNode& ikBone) {
    if (!ikBone.isIK) return;
    if (ikBone.ikTargetIdx < 0) return;

    auto* ikBoneMutable = model.getBone(ikBone.index);
    if (!ikBoneMutable) return;

    auto* targetBone = model.getBone(ikBone.ikTargetIdx);
    if (!targetBone) return;

    // Save best result for convergence
    std::vector<btQuaternion> bestIKRot;
    for (size_t i = 0; i < ikBone.ikLinks.size(); i++) {
        auto* lb = model.getBone(ikBone.ikLinks[i].boneIdx);
        bestIKRot.push_back(lb ? lb->ikRotate : btQuaternion::getIdentity());
    }
    float bestDist = 1e30f;

    // Track previous angles for decompose and plane mode angle for single-axis
    std::vector<btVector3> prevAngles(ikBone.ikLinks.size(), btVector3(0, 0, 0));
    std::vector<float> planeModeAngles(ikBone.ikLinks.size(), 0.0f);

    for (int loop = 0; loop < ikBone.ikLoopCount; loop++) {
        for (size_t li = 0; li < ikBone.ikLinks.size(); li++) {
            const auto& link = ikBone.ikLinks[li];
            auto* linkBone = model.getBone(link.boneIdx);
            if (!linkBone) continue;
            // Skip if chain bone is the target (would produce zero vectors)
            if (linkBone == targetBone) continue;

            node::updateLocalTransform(*linkBone);
            node::updateGlobalTransform(*linkBone, model);

            node::updateLocalTransform(*ikBoneMutable);
            node::updateGlobalTransform(*ikBoneMutable, model);

            // Check for single-axis constraint (knee-like)
            if (link.hasLimit) {
                bool xLim = (link.limitMin.x() != 0 || link.limitMax.x() != 0);
                bool yLim = (link.limitMin.y() != 0 || link.limitMax.y() != 0);
                bool zLim = (link.limitMin.z() != 0 || link.limitMax.z() != 0);

                if (xLim && !yLim && !zLim) {
                    solvePlane(model, ikBone, li, ikBone.ikRotationLimit, SolveAxis::X, loop, planeModeAngles[li]);
                    continue;
                } else if (!xLim && yLim && !zLim) {
                    solvePlane(model, ikBone, li, ikBone.ikRotationLimit, SolveAxis::Y, loop, planeModeAngles[li]);
                    continue;
                } else if (!xLim && !yLim && zLim) {
                    solvePlane(model, ikBone, li, ikBone.ikRotationLimit, SolveAxis::Z, loop, planeModeAngles[li]);
                    continue;
                }
            }

            btVector3 ikPos = ikBoneMutable->global.getOrigin();
            btVector3 targetPos = targetBone->global.getOrigin();

            // Transform to chain bone's local space (matches saba's SolveCore)
            btTransform invChain = linkBone->global.inverse();
            btVector3 chainIkPos = invChain * ikPos;
            btVector3 chainTargetPos = invChain * targetPos;

            if (chainIkPos.length2() < 1e-12f || chainTargetPos.length2() < 1e-12f) continue;

            btVector3 chainIkVec = chainIkPos.normalized();
            btVector3 chainTargetVec = chainTargetPos.normalized();

            float dot = chainTargetVec.dot(chainIkVec);
            dot = std::max(-1.0f, std::min(1.0f, dot));
            float angle = std::acos(dot);
            if (angle < 1.0e-3f) continue;
            angle = std::max(-ikBone.ikRotationLimit, std::min(ikBone.ikRotationLimit, angle));

            btVector3 cross = chainTargetVec.cross(chainIkVec).normalized();
            btQuaternion rot(btVector3(0, 0, 0), 0);
            rot.setRotation(cross, angle);

            // chainRot = ikRotate * animRotate * rot (matches saba)
            btQuaternion chainRot = linkBone->ikRotate * linkBone->animRotate * rot;

            if (link.hasLimit) {
                // Multi-axis: decompose with candidate selection, clamp, reconstruct
                float y, x, z;
                math::quaternionToEulerYxzCandidates(chainRot, prevAngles[li], y, x, z);

                x = clampAngle(x, link.limitMin.x(), link.limitMax.x());
                y = clampAngle(y, link.limitMin.y(), link.limitMax.y());
                z = clampAngle(z, link.limitMin.z(), link.limitMax.z());

                prevAngles[li] = btVector3(x, y, z);

                // Reconstruct clamped rotation
                chainRot = math::eulerToQuaternionYxz(y, x, z);
            }

            // ikRot = chainRot * inverse(animRotate) (matches saba)
            linkBone->ikRotate = chainRot * linkBone->animRotate.inverse();

            node::updateLocalTransform(*linkBone);
            node::updateGlobalTransform(*linkBone, model);
            node::updateChildTransforms(*linkBone, model);
        }

        // Check convergence
        node::updateLocalTransform(*ikBoneMutable);
        node::updateGlobalTransform(*ikBoneMutable, model);
        btVector3 ikEnd = ikBoneMutable->global.getOrigin();
        btVector3 tgtPos = targetBone->global.getOrigin();
        float dist = (ikEnd - tgtPos).length();

        if (dist < bestDist) {
            bestDist = dist;
            for (size_t i = 0; i < ikBone.ikLinks.size() && i < bestIKRot.size(); i++) {
                auto* lb = model.getBone(ikBone.ikLinks[i].boneIdx);
                if (lb) bestIKRot[i] = lb->ikRotate;
            }
        } else if (dist > bestDist * 1.5f) {
            break;
        }
    }
    // Apply best result
    for (size_t i = 0; i < ikBone.ikLinks.size() && i < bestIKRot.size(); i++) {
        auto* lb = model.getBone(ikBone.ikLinks[i].boneIdx);
        if (lb) lb->ikRotate = bestIKRot[i];
    }
}

} // namespace mmp::ik

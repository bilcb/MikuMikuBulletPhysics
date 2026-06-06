#include "solver.h"
#include "core/math/converter.h"
#include "core/node/transform.h"
#include "core/util/logger.h"
#include <algorithm>
#include <cmath>

namespace mmbp::ik {

static btQuaternion rotateFromTo(const btVector3 &from, const btVector3 &to) {
  btVector3 nf = from.normalized();
  btVector3 nt = to.normalized();
  btVector3 localW = nf.cross(nt);
  float dot = nf.dot(nt);

  if (std::abs(1.0f + dot) < 1.0e-7f) {

    btVector3 v(std::abs(from.x()), std::abs(from.y()), std::abs(from.z()));
    if (v.x() < v.y()) {
      if (v.x() < v.z())
        v = btVector3(1, 0, 0);
      else
        v = btVector3(0, 0, 1);
    } else {
      if (v.y() < v.z())
        v = btVector3(0, 1, 0);
      else
        v = btVector3(0, 0, 1);
    }
    btVector3 axis = from.cross(v).normalized();
    return btQuaternion(axis, SIMD_PI);
  }

  return btQuaternion(localW.x(), localW.y(), localW.z(), 1.0f + dot)
      .normalized();
}

static float clampAngle(float angle, float minAngle, float maxAngle) {
  if (minAngle == maxAngle)
    return minAngle;

  angle = mmbp::math::normalizeAngle(angle);
  float lo = mmbp::math::normalizeAngle(minAngle);
  float hi = mmbp::math::normalizeAngle(maxAngle);

  if (lo <= hi) {
    if (angle >= lo && angle <= hi)
      return angle;
  } else {
    if (angle >= lo || angle <= hi)
      return angle;
  }

  float diffLo = std::abs(angle - lo);
  float diffHi = std::abs(angle - hi);
  diffLo = std::min(diffLo, 2.0f * SIMD_PI - diffLo);
  diffHi = std::min(diffHi, 2.0f * SIMD_PI - diffHi);
  return (diffLo < diffHi) ? lo : hi;
}

enum class SolveAxis { X, Y, Z };

static void solvePlane(model::Model &model, const model::BoneNode &ikBone,
                       size_t linkIdx, SolveAxis axis, int iteration,
                       float &planeModeAngle) {
  const auto &link = ikBone.ikLinks[linkIdx];
  auto *linkBone = model.getBone(link.boneIdx);
  auto *ikBoneMutable = model.getBone(ikBone.index);
  auto *targetBone = model.getBone(ikBone.ikTargetIdx);
  if (!linkBone || !ikBoneMutable || !targetBone)
    return;

  int rotAxisIdx = 0;
  btVector3 rotAxis(1, 0, 0);

  switch (axis) {
  case SolveAxis::X:
    rotAxisIdx = 0;
    rotAxis = btVector3(1, 0, 0);
    break;
  case SolveAxis::Y:
    rotAxisIdx = 1;
    rotAxis = btVector3(0, 1, 0);
    break;
  case SolveAxis::Z:
    rotAxisIdx = 2;
    rotAxis = btVector3(0, 0, 1);
    break;
  }

  btVector3 ikPos = ikBoneMutable->global.getOrigin();
  btVector3 targetPos = targetBone->global.getOrigin();

  btTransform invChain = linkBone->global.inverse();
  btVector3 chainIkPos = invChain * ikPos;
  btVector3 chainTargetPos = invChain * targetPos;

  if (chainIkPos.length2() < 1e-12f || chainTargetPos.length2() < 1e-12f)
    return;

  btVector3 chainIkVec = chainIkPos.normalized();
  btVector3 chainTargetVec = chainTargetPos.normalized();

  float dot = chainTargetVec.dot(chainIkVec);
  dot = std::max(-1.0f, std::min(1.0f, dot));
  float angle = std::acos(dot);
  angle = std::max(-ikBone.ikRotationLimit,
                   std::min(ikBone.ikRotationLimit, angle));

  btQuaternion rot1(btVector3(0, 0, 0), 0);
  rot1.setRotation(rotAxis, angle);
  btVector3 targetVec1 = quatRotate(rot1, chainTargetVec);
  float dot1 = targetVec1.dot(chainIkVec);

  btQuaternion rot2(btVector3(0, 0, 0), 0);
  rot2.setRotation(rotAxis, -angle);
  btVector3 targetVec2 = quatRotate(rot2, chainTargetVec);
  float dot2 = targetVec2.dot(chainIkVec);

  float newAngle = planeModeAngle;
  if (dot1 > dot2) {
    newAngle += angle;
  } else {
    newAngle -= angle;
  }

  if (iteration == 0) {
    if (newAngle < link.limitMin[rotAxisIdx] ||
        newAngle > link.limitMax[rotAxisIdx]) {
      if (-newAngle > link.limitMin[rotAxisIdx] &&
          -newAngle < link.limitMax[rotAxisIdx]) {
        newAngle *= -1;
      } else {
        float halfRad =
            (link.limitMin[rotAxisIdx] + link.limitMax[rotAxisIdx]) * 0.5f;
        if (std::abs(halfRad - newAngle) > std::abs(halfRad + newAngle)) {
          newAngle *= -1;
        }
      }
    }
  }

  newAngle = std::max(link.limitMin[rotAxisIdx],
                      std::min(link.limitMax[rotAxisIdx], newAngle));
  planeModeAngle = newAngle;

  btQuaternion absRot(rotAxis, newAngle);
  linkBone->ikRotate = absRot * linkBone->animRotate.inverse();

  node::updateLocalTransform(*linkBone);
  node::updateGlobalTransform(*linkBone, model);
  node::updateChildTransforms(*linkBone, model);
}

const IkConfig *ikConfig() {
  static IkConfig cfg;
  return &cfg;
}

void solve(model::Model &model, const model::BoneNode &ikBone) {
  if (!ikBone.isIK)
    return;
  if (ikBone.ikTargetIdx < 0)
    return;

  auto *ikBoneMutable = model.getBone(ikBone.index);
  if (!ikBoneMutable)
    return;

  auto *targetBone = model.getBone(ikBone.ikTargetIdx);
  if (!targetBone)
    return;

  size_t nLinks = ikBone.ikLinks.size();
  std::vector<btQuaternion> bestIKRot;
  bestIKRot.reserve(nLinks);

  std::vector<btVector3> prevAngles(nLinks);
  for (size_t i = 0; i < nLinks; i++) {
    auto *lb = model.getBone(ikBone.ikLinks[i].boneIdx);
    if (lb) {
      bestIKRot.push_back(lb->ikRotate);

      float y, x, z;
      mmbp::math::quaternionToEulerYxz(lb->animRotate, y, x, z);
      prevAngles[i] = btVector3(x, y, z);
    } else {
      bestIKRot.push_back(btQuaternion::getIdentity());
      prevAngles[i] = btVector3(0, 0, 0);
    }
  }
  float bestDist = 1e30f;

  std::vector<float> planeModeAngles(nLinks, 0.0f);

  for (int loop = 0; loop < ikBone.ikLoopCount; loop++) {
    for (size_t li = 0; li < ikBone.ikLinks.size(); li++) {
      const auto &link = ikBone.ikLinks[li];
      auto *linkBone = model.getBone(link.boneIdx);
      if (!linkBone)
        continue;
      if (linkBone == targetBone)
        continue;

      node::updateLocalTransform(*linkBone);
      node::updateGlobalTransform(*linkBone, model);

      node::updateLocalTransform(*ikBoneMutable);
      node::updateGlobalTransform(*ikBoneMutable, model);

      if (link.hasLimit) {
        bool xLim = (link.limitMin.x() != 0 || link.limitMax.x() != 0);
        bool yLim = (link.limitMin.y() != 0 || link.limitMax.y() != 0);
        bool zLim = (link.limitMin.z() != 0 || link.limitMax.z() != 0);

        if (xLim && !yLim && !zLim) {
          solvePlane(model, ikBone, li, SolveAxis::X, loop,
                     planeModeAngles[li]);
          continue;
        } else if (!xLim && yLim && !zLim) {
          solvePlane(model, ikBone, li, SolveAxis::Y, loop,
                     planeModeAngles[li]);
          continue;
        } else if (!xLim && !yLim && zLim) {
          solvePlane(model, ikBone, li, SolveAxis::Z, loop,
                     planeModeAngles[li]);
          continue;
        }
      }

      btVector3 ikPos = ikBoneMutable->global.getOrigin();
      btVector3 targetPos = targetBone->global.getOrigin();

      btTransform invChain = linkBone->global.inverse();
      btVector3 chainIkPos = invChain * ikPos;
      btVector3 chainTargetPos = invChain * targetPos;

      if (chainIkPos.length2() < 1e-12f || chainTargetPos.length2() < 1e-12f)
        continue;

      btVector3 chainIkVec = chainIkPos.normalized();
      btVector3 chainTargetVec = chainTargetPos.normalized();

      float dot = chainTargetVec.dot(chainIkVec);
      dot = std::max(-1.0f, std::min(1.0f, dot));
      float angle = std::acos(dot);
      if (angle < 1.0e-3f)
        continue;
      angle = std::max(-ikBone.ikRotationLimit,
                       std::min(ikBone.ikRotationLimit, angle));

      btQuaternion rot = rotateFromTo(chainTargetVec, chainIkVec);

      btScalar rotAngle = rot.getAngle();
      if (std::abs(rotAngle) > std::abs(angle)) {
        btVector3 rotAxis = rot.getAxis();
        rot.setRotation(rotAxis, angle);
      }

      btScalar stepAngle = rot.getAngle();
      float maxStepAngle = ikConfig()->maxStepAngle;
      if (stepAngle > maxStepAngle) {
        btVector3 stepAxis = rot.getAxis();
        rot.setRotation(stepAxis, maxStepAngle);
      }

      btQuaternion chainRot = linkBone->ikRotate * linkBone->animRotate * rot;

      if (link.hasLimit) {
        float y, x, z;
        math::quaternionToEulerYxzCandidates(chainRot, prevAngles[li], y, x, z);

        x = clampAngle(x, link.limitMin.x(), link.limitMax.x());
        y = clampAngle(y, link.limitMin.y(), link.limitMax.y());
        z = clampAngle(z, link.limitMin.z(), link.limitMax.z());

        prevAngles[li] = btVector3(x, y, z);

        chainRot = math::eulerToQuaternionYxz(y, x, z);
      }

      linkBone->ikRotate = chainRot * linkBone->animRotate.inverse();

      node::updateLocalTransform(*linkBone);
      node::updateGlobalTransform(*linkBone, model);
      node::updateChildTransforms(*linkBone, model);
    }

    node::updateLocalTransform(*ikBoneMutable);
    node::updateGlobalTransform(*ikBoneMutable, model);
    btVector3 ikEnd = ikBoneMutable->global.getOrigin();
    btVector3 tgtPos = targetBone->global.getOrigin();
    float dist = (ikEnd - tgtPos).length();

    if (dist < bestDist) {
      bestDist = dist;
      for (size_t i = 0; i < ikBone.ikLinks.size() && i < bestIKRot.size();
           i++) {
        auto *lb = model.getBone(ikBone.ikLinks[i].boneIdx);
        if (lb)
          bestIKRot[i] = lb->ikRotate;
      }
    } else if (dist > bestDist * ikConfig()->distDegradeRatio) {
      break;
    }
  }
  for (size_t i = 0; i < ikBone.ikLinks.size() && i < bestIKRot.size(); i++) {
    auto *lb = model.getBone(ikBone.ikLinks[i].boneIdx);
    if (!lb)
      continue;

    const btQuaternion &r = bestIKRot[i];
    if (std::isfinite(r.w()) && std::isfinite(r.x()) && std::isfinite(r.y()) &&
        std::isfinite(r.z())) {
      lb->ikRotate = r;
    } else {
      g_logger.warn("[MMBP] IK solve produced non-finite rotation for bone "
                    "'%s' (idx=%d), keeping previous frame's value",
                    lb->name.c_str(), lb->index);
    }
  }
}

} // namespace mmbp::ik

#pragma once
#include <btBulletDynamicsCommon.h>

namespace mmbp::math {

btVector3 applyScale(const btVector3 &v, float scale);
btVector3 removeScale(const btVector3 &v, float scale);

btQuaternion eulerToQuaternionYxz(float y, float x, float z);
btMatrix3x3 eulerToMatrix3x3Yxz(float y, float x, float z);
void computeBasicEulerYxz(const btQuaternion &q, float &outY, float &outX,
                          float &outZ, float &outSinY);

void quaternionToEulerYxz(const btQuaternion &q, float &outY, float &outX,
                          float &outZ);
void quaternionToEulerYxzCandidates(const btQuaternion &q,
                                    const btVector3 &before, float &outY,
                                    float &outX, float &outZ);

float normalizeAngle(float angle);
float diffAngle(float a, float b);

inline btQuaternion convertRotationToLocal(const btQuaternion &baseQ,
                                           const btQuaternion &rot) {
  return baseQ * rot * baseQ.inverse();
}

inline btVector3 scaleVector(const btVector3 &v, float s) { return v * s; }
inline btVector3 unscaleVector(const btVector3 &v, float s) {
  return std::abs(s) < 1e-12f ? v : v / s;
}

} // namespace mmbp::math

#pragma once
#include <btBulletDynamicsCommon.h>

namespace mmp::math {

// ---- Unit conversion (MMD cm ↔ Blender m) ----
btVector3 applyScale(const btVector3& v, float scale);
btVector3 removeScale(const btVector3& v, float scale);

// ---- MMD Euler YXZ ↔ Quaternion ----
btQuaternion eulerToQuaternionYxz(float y, float x, float z);
btMatrix3x3  eulerToMatrix3x3Yxz(float y, float x, float z);
void         quaternionToEulerYxz(const btQuaternion& q, float& outY, float& outX, float& outZ);
void         quaternionToEulerYxzCandidates(const btQuaternion& q, const btVector3& before, float& outY, float& outX, float& outZ);

} // namespace mmp::math

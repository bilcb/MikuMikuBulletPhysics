#include "converter.h"
#include <algorithm>
#include <cmath>

namespace mmbp::math {

float normalizeAngle(float angle) {
  angle = std::fmod(angle, SIMD_PI * 2.0f);
  if (angle >= SIMD_PI)
    angle -= SIMD_PI * 2.0f;
  if (angle < -SIMD_PI)
    angle += SIMD_PI * 2.0f;
  return angle;
}

float diffAngle(float a, float b) {
  float diff = normalizeAngle(a) - normalizeAngle(b);
  if (diff > SIMD_PI)
    return diff - SIMD_PI * 2.0f;
  if (diff < -SIMD_PI)
    return diff + SIMD_PI * 2.0f;
  return diff;
}

btVector3 applyScale(const btVector3 &v, float scale) { return v * scale; }

btVector3 removeScale(const btVector3 &v, float scale) {

  if (std::abs(scale) < 1.0e-10f)
    return v;
  return v * (1.0f / scale);
}

void computeBasicEulerYxz(const btQuaternion &q, float &outY, float &outX,
                          float &outZ, float &outSinY) {
  btScalar sqx = q.x() * q.x();
  btScalar sqy = q.y() * q.y();
  btScalar sqz = q.z() * q.z();
  btScalar m02 = 2.0f * (q.x() * q.z() - q.w() * q.y());
  float sinY = -m02;
  sinY = std::max(-1.0f, std::min(1.0f, sinY));
  btScalar m22 = 1.0f - 2.0f * (sqx + sqy);
  btScalar m12 = 2.0f * (q.y() * q.z() + q.w() * q.x());
  btScalar m00 = 1.0f - 2.0f * (sqy + sqz);
  btScalar m01 = 2.0f * (q.x() * q.y() + q.w() * q.z());
  outX = std::atan2(m12, m22);
  outY = std::asin(sinY);
  outZ = std::atan2(m01, m00);
  outSinY = sinY;
}

btQuaternion eulerToQuaternionYxz(float y, float x, float z) {
  btQuaternion qy(btVector3(0, 1, 0), y);
  btQuaternion qx(btVector3(1, 0, 0), x);
  btQuaternion qz(btVector3(0, 0, 1), z);
  return qy * qx * qz;
}

btMatrix3x3 eulerToMatrix3x3Yxz(float y, float x, float z) {
  btMatrix3x3 mat;
  mat.setRotation(eulerToQuaternionYxz(y, x, z));
  return mat;
}

void quaternionToEulerYxz(const btQuaternion &q, float &outY, float &outX,
                          float &outZ) {
  float sinY;
  computeBasicEulerYxz(q, outY, outX, outZ, sinY);
  const float e = 1.0e-6f;
  if (std::abs(1.0f - std::abs(sinY)) < e) {
    float sx = std::sin(outX);
    float sz = std::sin(outZ);
    if (std::abs(sx) < std::abs(sz)) {
      float cx = std::cos(outX);
      if (cx > 0) {
        outX = 0;
        btScalar m10 = 2.0f * (q.x() * q.y() - q.w() * q.z());
        outZ = std::asin(-m10);
      } else {
        outX = SIMD_PI;
        btScalar m10 = 2.0f * (q.x() * q.y() - q.w() * q.z());
        outZ = std::asin(m10);
      }
    } else {
      float cz = std::cos(outZ);
      if (cz > 0) {
        outZ = 0;
        btScalar m21 = 2.0f * (q.y() * q.z() - q.w() * q.x());
        outX = std::asin(-m21);
      } else {
        outZ = SIMD_PI;
        btScalar m21 = 2.0f * (q.y() * q.z() - q.w() * q.x());
        outX = std::asin(m21);
      }
    }
  }
}

void quaternionToEulerYxzCandidates(const btQuaternion &q,
                                    const btVector3 &before, float &outY,
                                    float &outX, float &outZ) {
  float rx, ry, rz;
  float sinY;
  computeBasicEulerYxz(q, ry, rx, rz, sinY);
  const float e = 1.0e-6f;
  if (std::abs(1.0f - std::abs(sinY)) < e) {

    float sx = std::sin(before.x());
    float sz = std::sin(before.z());
    if (std::abs(sx) < std::abs(sz)) {
      float cx = std::cos(before.x());
      if (cx > 0) {
        rx = 0;
        btScalar m10 = 2.0f * (q.x() * q.y() - q.w() * q.z());
        rz = std::asin(-m10);
      } else {
        rx = SIMD_PI;
        btScalar m10 = 2.0f * (q.x() * q.y() - q.w() * q.z());
        rz = std::asin(m10);
      }
    } else {
      float cz = std::cos(before.z());
      if (cz > 0) {
        rz = 0;
        btScalar m21 = 2.0f * (q.y() * q.z() - q.w() * q.x());
        rx = std::asin(-m21);
      } else {
        rz = SIMD_PI;
        btScalar m21 = 2.0f * (q.y() * q.z() - q.w() * q.x());
        rx = std::asin(m21);
      }
    }
  }

  outX = rx;
  outY = ry;
  outZ = rz;

  float errX = std::abs(diffAngle(rx, before.x()));
  float errY = std::abs(diffAngle(ry, before.y()));
  float errZ = std::abs(diffAngle(rz, before.z()));
  float minErr = errX + errY + errZ;

  const float pi = SIMD_PI;

  static const btVector3 tests[] = {
      btVector3(pi, pi, pi),   btVector3(pi, pi, -pi),
      btVector3(pi, -pi, pi),  btVector3(pi, -pi, -pi),
      btVector3(-pi, pi, pi),  btVector3(-pi, pi, -pi),
      btVector3(-pi, -pi, pi), btVector3(-pi, -pi, -pi),
  };

  for (const auto &t : tests) {
    float err = std::abs(diffAngle(rx + t.x(), before.x())) +
                std::abs(diffAngle(pi - ry + t.y(), before.y())) +
                std::abs(diffAngle(rz + t.z(), before.z()));
    if (err < minErr) {
      minErr = err;
      outX = rx + t.x();
      outY = pi - ry + t.y();
      outZ = rz + t.z();
    }
  }
}

} // namespace mmbp::math

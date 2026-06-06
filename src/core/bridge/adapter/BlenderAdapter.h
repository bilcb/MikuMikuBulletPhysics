#pragma once
#include <btBulletDynamicsCommon.h>

namespace mmbp::bridge {

class BlenderAdapter {
public:
  static btMatrix3x3 computeBoneRestMatrix(const btMatrix3x3 &boneLocal);

  static btVector3 computeBoneRestOffset(const btMatrix3x3 &boneLocal,
                                         const btVector3 &restPos);
};

} // namespace mmbp::bridge

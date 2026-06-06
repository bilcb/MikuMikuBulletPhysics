#include "BlenderAdapter.h"

namespace mmbp::bridge {

btMatrix3x3
BlenderAdapter::computeBoneRestMatrix(const btMatrix3x3 &boneLocal) {

  const btVector3 &r0 = boneLocal.getRow(0);
  const btVector3 &r1 = boneLocal.getRow(2);
  const btVector3 &r2 = boneLocal.getRow(1);
  btMatrix3x3 swapped(r0.x(), r0.y(), r0.z(), r1.x(), r1.y(), r1.z(), r2.x(),
                      r2.y(), r2.z());
  return swapped.transpose();
}

btVector3 BlenderAdapter::computeBoneRestOffset(const btMatrix3x3 &boneLocal,
                                                const btVector3 &restPos) {
  return boneLocal.transpose() * restPos;
}

} // namespace mmbp::bridge

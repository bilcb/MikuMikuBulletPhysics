#include "BlenderAdapter.h"

namespace mmp::bridge {

static const btMatrix3x3 kYupToZup(
    1, 0, 0,
    0, 0, 1,
    0, 1, 0
);
static const btMatrix3x3 kZupToYup(
    1, 0, 0,
    0, 0, 1,
    0, 1, 0
);

void BlenderAdapter::initialize() {}

btVector3 BlenderAdapter::mmdToBlenderPosition(const btVector3& v) { return kYupToZup * v; }
btVector3 BlenderAdapter::blenderToMmdPosition(const btVector3& v) { return kZupToYup * v; }

btQuaternion BlenderAdapter::mmdToBlenderRotation(const btQuaternion& q, const btMatrix3x3& convMat) {
    btQuaternion qConv;
    convMat.getRotation(qConv);
    return qConv * q * qConv.inverse();
}

btQuaternion BlenderAdapter::blenderToMmdRotation(const btQuaternion& q, const btMatrix3x3& convMat) {
    btQuaternion qConv;
    convMat.getRotation(qConv);
    return qConv.inverse() * q * qConv;
}

btVector3 BlenderAdapter::mmdToSabaPosition(const btVector3& v) {
    return btVector3(v.x(), v.y(), -v.z());
}

btQuaternion BlenderAdapter::mmdToSabaRotation(const btQuaternion& q) {
    btQuaternion r = q;
    r.setZ(-r.z());
    return r;
}

btMatrix3x3 BlenderAdapter::computeBoneRestMatrix(const btMatrix3x3& boneLocal) {
    const btVector3& r0 = boneLocal.getRow(0);
    const btVector3& r1 = boneLocal.getRow(2);
    const btVector3& r2 = boneLocal.getRow(1);
    btMatrix3x3 swapped(
        r0.x(), r0.y(), r0.z(),
        r1.x(), r1.y(), r1.z(),
        r2.x(), r2.y(), r2.z()
    );
    return swapped.transpose();
}

btVector3 BlenderAdapter::computeBoneRestOffset(const btMatrix3x3& boneLocal, const btVector3& restPos) {
    return boneLocal.transpose() * restPos;
}

btVector3 BlenderAdapter::applyRestMatrixPosition(const btVector3& pos, const btMatrix3x3& rm, const btVector3& offset) {
    return rm * pos - offset;
}

btQuaternion BlenderAdapter::applyRestMatrixRotation(const btQuaternion& rot, const btMatrix3x3& rm) {
    // Matches mmd_tools BoneConverter.convert_rotation():
    // q = rm.to_quaternion(); result = q * rot * q.conjugated()
    // This is a similarity transform (basis change), NOT a direct rotation.
    btMatrix3x3 rRot;
    rRot.setRotation(rot);
    rRot = rm * rRot * rm.transpose();
    btQuaternion result;
    rRot.getRotation(result);
    return result;
}

} // namespace mmp::bridge

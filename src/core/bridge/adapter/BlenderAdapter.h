#pragma once
#include <btBulletDynamicsCommon.h>

namespace mmp::bridge {

class BlenderAdapter {
public:
    static void initialize();

    // ---- Space conversions (MMD ↔ Blender) ----
    static btVector3   mmdToBlenderPosition(const btVector3& v);
    static btVector3   blenderToMmdPosition(const btVector3& v);
    static btQuaternion mmdToBlenderRotation(const btQuaternion& q, const btMatrix3x3& convMat = btMatrix3x3::getIdentity());
    static btQuaternion blenderToMmdRotation(const btQuaternion& q, const btMatrix3x3& convMat = btMatrix3x3::getIdentity());

    // ---- MMD ↔ saba coordinate ----
    static btVector3   mmdToSabaPosition(const btVector3& v);
    static btQuaternion mmdToSabaRotation(const btQuaternion& q);

    // ---- Rest matrix (Blender bone ↔ MMD) ----
    static btMatrix3x3  computeBoneRestMatrix(const btMatrix3x3& boneLocal);
    static btVector3    computeBoneRestOffset(const btMatrix3x3& boneLocal, const btVector3& restPos);
    static btVector3    applyRestMatrixPosition(const btVector3& pos, const btMatrix3x3& rm, const btVector3& offset);
    static btQuaternion applyRestMatrixRotation(const btQuaternion& rot, const btMatrix3x3& rm);
};

} // namespace mmp::bridge

#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <btBulletDynamicsCommon.h>

namespace mmp::model {

struct BoneNode {
    struct IKLink {
        int32_t boneIdx = -1;
        bool hasLimit = false;
        btVector3 limitMin = btVector3(0,0,0);
        btVector3 limitMax = btVector3(0,0,0);
    };

    uint32_t    index = 0;
    std::string name;
    int32_t     parentIdx = -1;
    btVector3   restPosition = btVector3(0,0,0);
    btVector3   animTranslate = btVector3(0,0,0);
    btQuaternion animRotate   = btQuaternion(0,0,0,1);
    btVector3 ikTranslate   = btVector3(0,0,0);
    btQuaternion ikRotate     = btQuaternion(0,0,0,1);
    btVector3 baseAnimTranslate = btVector3(0,0,0);
    btQuaternion baseAnimRotate = btQuaternion(0,0,0,1);
    btTransform local   = btTransform::getIdentity();
    btTransform global  = btTransform::getIdentity();
    btTransform inverseInit = btTransform::getIdentity();
    bool isIK = false;
    bool ikEnabled = true;
    bool transAfterPhys = false;
    int32_t deformDepth = 0;
    bool hasAppendRotate = false;
    bool hasAppendTranslate = false;
    int32_t appendBoneIdx = -1;
    float appendWeight = 1.0f;
    btVector3 appendTranslate = btVector3(0,0,0);
    btQuaternion appendRotate = btQuaternion(0,0,0,1);
    int32_t ikTargetIdx = -1;
    int32_t ikLoopCount = 0;
    float   ikRotationLimit = 1.0f;
    std::vector<IKLink> ikLinks;
    std::vector<uint32_t> children;
};

} // namespace mmp::model

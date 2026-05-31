#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <btBulletDynamicsCommon.h>

namespace mmp::pmx {

struct PMXBone {
    std::string name;
    std::string nameEn;
    btVector3   position = btVector3(0,0,0);
    int32_t     parentIdx = -1;
    int32_t     deformDepth = 0;
    uint16_t    flags = 0;
    int32_t     displayConnection = -1;
    int32_t     transformOrder = 0;
    int32_t     externalTransKey = -1;
    btVector3   displayOffset = btVector3(0,0,0);
    bool        isControllable = true;
    bool        isRotatable = true;
    bool        isMovable = true;
    bool        visible = true;
    bool        transAfterPhys = false;
    bool        hasAdditionalRotate = false;
    bool        hasAdditionalLocate = false;
    bool        appendLocal = false;
    int32_t     appendBoneIdx = -1;
    float       appendWeight = 1.0f;
    bool        isIK = false;
    int32_t     ikTargetIdx = -1;
    int32_t     ikLoopCount = 0;
    float       ikRotationLimit = 1.0f;
    struct IKLink {
        int32_t boneIdx;
        bool    hasLimit;
        btVector3 limitMin;
        btVector3 limitMax;
    };
    std::vector<IKLink> ikLinks;
    btVector3   localAxisX = btVector3(0,0,0);
    btVector3   localAxisZ = btVector3(0,0,0);
    btVector3   fixedAxis = btVector3(0,0,0);
    bool        hasLocalAxes = false;
    bool        hasFixedAxis = false;
};

struct PMXRigidBody {
    std::string name;
    std::string nameEn;
    int32_t     boneIdx = -1;
    uint8_t     shapeType = 0;
    uint8_t     group = 0;
    uint16_t    groupMask = 0;
    btVector3   size = btVector3(0,0,0);
    btVector3   position = btVector3(0,0,0);
    btQuaternion rotation = btQuaternion(0,0,0,1);
    btVector3   eulerRotation = btVector3(0,0,0);  // raw PMX Euler YXZ
    float       mass = 0;
    float       linearDamping = 0;
    float       angularDamping = 0;
    float       restitution = 0;
    float       friction = 0;
    uint8_t     mode = 0;
};

struct PMXJoint {
    std::string name;
    std::string nameEn;
    uint8_t     mode = 0;
    int32_t     rigidBodyAIdx = -1;
    int32_t     rigidBodyBIdx = -1;
    btVector3   position = btVector3(0,0,0);
    btQuaternion rotation = btQuaternion(0,0,0,1);
    btVector3   linearLowerLimit = btVector3(0,0,0);
    btVector3   linearUpperLimit = btVector3(0,0,0);
    btVector3   angularLowerLimit = btVector3(0,0,0);
    btVector3   angularUpperLimit = btVector3(0,0,0);
    btVector3   springTranslate = btVector3(0,0,0);
    btVector3   springRotate = btVector3(0,0,0);
};

struct PMXMorph {
    std::string name;
    std::string nameEn;
    uint8_t     category = 0;
    struct VertexOffset   { int32_t idx; btVector3 offset; };
    struct BoneOffset     { int32_t idx; btVector3 translation; btQuaternion rotation; };
    struct MaterialOffset { int32_t idx; uint8_t op; float diffuse[4]; float specular[3]; float shininess; float ambient[3]; float edgeColor[4]; float edgeSize; float texture[4]; float sphere[4]; float toon[4]; };
    struct GroupOffset    { int32_t idx; float factor; };
    struct UVOffset       { int32_t idx; float offset[4]; };
    struct FlipOffset     { int32_t idx; float factor; };
    struct ImpulseOffset  { int32_t idx; bool local; btVector3 velocity; btVector3 torque; };
    std::vector<VertexOffset>   vertexOffsets;
    std::vector<BoneOffset>     boneOffsets;
    std::vector<MaterialOffset> materialOffsets;
    std::vector<GroupOffset>    groupOffsets;
    std::vector<UVOffset>       uvOffsets;
    std::vector<FlipOffset>     flipOffsets;
    std::vector<ImpulseOffset>  impulseOffsets;
};

struct PMXDisplayFrame {
    std::string name;
    bool isSpecial = false;
    struct Target { uint8_t type; int32_t index; };
    std::vector<Target> targets;
};

struct PMXData {
    std::string name;
    std::string comment;
    std::vector<PMXBone>      bones;
    std::vector<PMXRigidBody> rigidBodies;
    std::vector<PMXJoint>     joints;
    std::vector<PMXMorph>     morphs;
    std::vector<PMXDisplayFrame> displayFrames;
};

} // namespace mmp::pmx

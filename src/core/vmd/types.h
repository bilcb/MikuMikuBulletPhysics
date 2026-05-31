#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <btBulletDynamicsCommon.h>

namespace mmp::vmd {

struct VMDKeyframe {
    std::string boneName;
    uint32_t    frame = 0;
    btVector3   translation = btVector3(0,0,0);
    btQuaternion rotation    = btQuaternion(0,0,0,1);
    uint8_t     interpX[16] = {0};
    uint8_t     interpY[16] = {0};
    uint8_t     interpZ[16] = {0};
    uint8_t     interpRot[16]= {0};
};

struct MorphKeyframe {
    std::string name;
    uint32_t    frame = 0;
    float       weight = 0;
};

struct IKKeyframe {
    uint32_t frame = 0;
    bool     show = true;
    struct IKState { std::string name; bool enabled; };
    std::vector<IKState> states;
};

struct VMDData {
    std::string name;
    std::vector<VMDKeyframe>   keyframes;
    std::vector<MorphKeyframe> morphKeys;
    std::vector<IKKeyframe>    ikKeys;
};

} // namespace mmp::vmd

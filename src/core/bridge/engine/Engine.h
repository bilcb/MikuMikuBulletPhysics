#pragma once
#include "PhysicsPipeline.h"
#include "core/pmx/types.h"
#include "core/vmd/types.h"
#include "core/physics/world.h"
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

namespace mmp::model { class Model; class BoneNode; }

namespace mmp::bridge {

class Engine {
public:
    Engine();
    ~Engine();

    bool loadPMX(const uint8_t* data, int size);
    bool loadPMXFile(const char* path);
    bool loadVMD(const uint8_t* data, int size);
    bool loadVMDFile(const char* path);
    bool buildPhysics();
    int step(float frame, float delta, float* outLocs, float* outQuats, int boneCount);

    void setPhysicsConfig(const physics::Config& config);
    const physics::Config& getPhysicsConfig() const { return m_physicsConfig; }
    void setBoneRestInfo(const float* mat3Array, int count);
    void setBoneRestPositions(const float* offsetArray, int count);

    int getBoneCount() const;
    int getBoneName(int idx, char* out, int maxLen) const;
    int getRigidBodyCount() const;
    int getJointCount() const;
    const char* getLastError() const;
    const model::BoneNode* getModelBone(int idx) const;
    physics::RigidBody* getRigidBody(int idx);
    physics::World* getPhysicsWorld();

    void evalAnimation(float frame);
    void solveIK();
    void stepPhysics(float frame, float delta);
    void getBoneWorldMat4(int idx, float* outMat4) const;
    int getIKCount() const;
    void getSolverInfo(float* out) const;

private:
    std::unique_ptr<model::Model> m_model;
    std::unique_ptr<physics::World> m_world;
    pmx::PMXData m_pmxData;
    vmd::VMDData m_vmdData;
    std::vector<btMatrix3x3> m_restMatrices;
    std::vector<btVector3> m_restPosOffsets;
    physics::Config m_physicsConfig;
    bool m_modelLoaded = false;
    bool m_physicsBuilt = false;
    PhysicsPipeline m_pipeline;
    mutable std::string m_lastError;
};

} // namespace mmp::bridge

#pragma once
#include "core/model/model.h"
#include "core/pmx/types.h"
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <memory>

namespace mmp::physics {

// Collision filter matching saba's MMDFilterCallback
struct MMDFilterCallback : public btOverlapFilterCallback {
    std::vector<btBroadphaseProxy*> m_nonFilterProxies;
    bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override {
        for (auto* p : m_nonFilterProxies) { if (proxy0 == p || proxy1 == p) return true; }
        bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
        collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
        return collides;
    }
};

struct Config {
    btVector3 gravity = btVector3(0, -98.0f, 0);
    int solverIterations = 10;
    float fixedTimestep = 1.0f / 120.0f;
    int maxSubSteps = 10;
    float scale = 0.08f;
};

// Base motion state for MMD rigid bodies
class MMDMotionState : public btMotionState {
public:
    virtual void reset() = 0;
    virtual void reflectGlobalTransform() = 0;
};

// Kinematic mode: follows bone animation, never simulated
class KinematicMotionState : public MMDMotionState {
public:
    KinematicMotionState(const model::BoneNode* node, const btTransform& offset, float scale);
    void getWorldTransform(btTransform& worldTransform) const override;
    void setWorldTransform(const btTransform& worldTransform) override;
    void reset() override;
    void reflectGlobalTransform() override;
private:
    const model::BoneNode* m_node;
    btTransform m_offset;
    float m_scale;
};

// Dynamic mode: fully simulated, writes back to bone
class DynamicMotionState : public MMDMotionState {
public:
    DynamicMotionState(const model::BoneNode* node, const btTransform& offset, float scale);
    void getWorldTransform(btTransform& worldTransform) const override;
    void setWorldTransform(const btTransform& worldTransform) override;
    void reset() override;
    void reflectGlobalTransform() override;
private:
    const model::BoneNode* m_node;
    btTransform m_offset;
    btTransform m_invOffset;
    btTransform m_transform;
    float m_scale;
};

// Dynamic + bone merge: simulated but preserves bone Y position
class DynamicBoneMergeMotionState : public MMDMotionState {
public:
    DynamicBoneMergeMotionState(const model::BoneNode* node, const btTransform& offset, float scale);
    void getWorldTransform(btTransform& worldTransform) const override;
    void setWorldTransform(const btTransform& worldTransform) override;
    void reset() override;
    void reflectGlobalTransform() override;
private:
    const model::BoneNode* m_node;
    btTransform m_offset;
    btTransform m_invOffset;
    btTransform m_transform;
    float m_scale;
};

class RigidBody {
public:
    std::unique_ptr<btRigidBody> body;
    std::unique_ptr<btCollisionShape> shape;
    std::unique_ptr<MMDMotionState> activeMotionState;
    std::unique_ptr<MMDMotionState> kinematicMotionState;
    std::unique_ptr<btDefaultMotionState> defaultMotionState;
    int boneIdx = -1;
    uint16_t group = 0;
    uint16_t groupMask = 0;
    btTransform offsetMat = btTransform::getIdentity();
    btTransform boneWorldInit = btTransform::getIdentity();
    uint8_t mode = 0;
};

class World {
public:
    World() = default;
    ~World();
    bool create(const Config& cfg);
    void destroy();
    int addRigidBody(const pmx::PMXRigidBody& rbData, int boneIdx, float scale,
                    const model::BoneNode* bone = nullptr);
    void addJoint(const pmx::PMXJoint& jointData, const RigidBody* rbA, const RigidBody* rbB);
    void step(float delta);
    void resetPhysics(model::Model& model);
    void syncBoneTransforms(model::Model& model);
    void setActivation(int idx, bool active);
    RigidBody* getRigidBody(int idx) { return idx < (int)m_rigidBodies.size() ? &m_rigidBodies[idx] : nullptr; }
    int rigidBodyCount() const { return (int)m_rigidBodies.size(); }
    int jointCount() const { return (int)m_joints.size(); }
    btDiscreteDynamicsWorld* getWorld() { return m_dynamicsWorld.get(); }
    bool boneHasRigidBody(int boneIdx) const { return boneIdx >= 0 && boneIdx < (int)m_boneHasRb.size() && m_boneHasRb[boneIdx]; }
    void cascadeChildTransforms(model::Model& model, int boneIdx);

private:
    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionCfg;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_dynamicsWorld;
    std::unique_ptr<btRigidBody> m_groundBody;
    std::unique_ptr<btCollisionShape> m_groundShape;
    std::unique_ptr<btDefaultMotionState> m_groundMotionState;
    std::unique_ptr<MMDFilterCallback> m_filterCallback;
    std::vector<RigidBody> m_rigidBodies;
    std::vector<std::unique_ptr<btTypedConstraint>> m_joints;
    Config m_config;
    bool m_created = false;
    std::vector<char> m_boneHasRb; // boneIdx → has rigid body
};

} // namespace mmp::physics

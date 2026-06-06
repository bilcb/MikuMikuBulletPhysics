#pragma once
#include "core/model/model.h"
#include "core/pmx/types.h"
#include <btBulletDynamicsCommon.h>
#include <memory>
#include <vector>

namespace mmbp::physics {

struct MMDFilterCallback : public btOverlapFilterCallback {
  std::vector<btBroadphaseProxy *> m_nonFilterProxies;
  bool needBroadphaseCollision(btBroadphaseProxy *proxy0,
                               btBroadphaseProxy *proxy1) const override {
    for (auto *p : m_nonFilterProxies) {
      if (proxy0 == p || proxy1 == p)
        return true;
    }
    bool collides =
        (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
    collides = collides &&
               (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
    return collides;
  }
};

struct Config {
  btVector3 gravity = btVector3(0, -98.0f, 0);
  int solverIterations = 10;
    float fixedTimestep = 1.0f / 60.0f;
    int maxSubSteps = 6;
  float scale = 0.08f;
  float springDamping = 0.3f;

  float springDampingLinear[3] = {0.0f, 0.0f, 0.0f};
  float springDampingAngular[3] = {0.0f, 0.0f, 0.0f};

  bool enableSleepDeactivation = false;
};

class MMDMotionState : public btMotionState {
public:
  virtual void reset() = 0;
  virtual void reflectGlobalTransform() = 0;
  virtual void clearNode() = 0;

protected:
  void resetFromNode(model::BoneNode *node, const btTransform &offset,
                     float scale, btTransform &outTransform);
};

class KinematicMotionState : public MMDMotionState {
public:
  KinematicMotionState(model::BoneNode *node, const btTransform &offset,
                       float scale);
  void getWorldTransform(btTransform &worldTransform) const override;
  void setWorldTransform(const btTransform &worldTransform) override;
  void reset() override {}
  void reflectGlobalTransform() override {}
  void clearNode() override { m_node = nullptr; }

private:
  model::BoneNode *m_node;
  btTransform m_offset;
  float m_scale;
};

class DynamicMotionState : public MMDMotionState {
public:
  DynamicMotionState(model::BoneNode *node, const btTransform &offset,
                     float scale);
  void getWorldTransform(btTransform &worldTransform) const override;
  void setWorldTransform(const btTransform &worldTransform) override;
  void reset() override;
  void reflectGlobalTransform() override;
  void clearNode() override { m_node = nullptr; }

protected:
  model::BoneNode *m_node;
  btTransform m_offset;
  btTransform m_invOffset;
  btTransform m_transform;
  float m_scale;
};

class DynamicBoneMergeMotionState : public DynamicMotionState {
public:
  DynamicBoneMergeMotionState(model::BoneNode *node, const btTransform &offset,
                              float scale);
  void reflectGlobalTransform() override;
};

class DefaultMotionState : public MMDMotionState {
public:
  DefaultMotionState(const btTransform &initial) : m_transform(initial) {}
  void getWorldTransform(btTransform &worldTransform) const override {
    worldTransform = m_transform;
  }
  void setWorldTransform(const btTransform &worldTransform) override {
    m_transform = worldTransform;
  }
  void reset() override {}
  void reflectGlobalTransform() override {}
  void clearNode() override {}

private:
  btTransform m_transform;
};

class RigidBody {
public:
  std::unique_ptr<btRigidBody> body;
  std::unique_ptr<btCollisionShape> shape;
  std::unique_ptr<MMDMotionState> activeMotionState;
  std::unique_ptr<MMDMotionState> kinematicMotionState;
  int boneIdx = -1;
  uint16_t group = 0;
  uint16_t groupMask = 0;
  btTransform offsetMat = btTransform::getIdentity();
  uint8_t mode = 0;
};

class World {
public:
  World() = default;
  ~World();
  bool create(const Config &cfg);
  void destroy();
  int addRigidBody(const pmx::PMXRigidBody &rbData, int boneIdx, float scale,
                   model::BoneNode *bone = nullptr);
  void addJoint(const pmx::PMXJoint &jointData, const RigidBody *rbA,
                const RigidBody *rbB);
  void step(float delta);
  void resetPhysics(model::Model &model);
  void syncBoneTransforms(model::Model &model);
  void setActivation(int idx, bool active);
  void setAllDeactivated();
  RigidBody *getRigidBody(int idx) {
    return idx < (int)m_rigidBodies.size() ? &m_rigidBodies[idx] : nullptr;
  }
  int rigidBodyCount() const { return (int)m_rigidBodies.size(); }
  int jointCount() const { return (int)m_joints.size(); }
  btDiscreteDynamicsWorld *getWorld() { return m_dynamicsWorld.get(); }
  bool boneHasRigidBody(int boneIdx) const {
    return boneIdx >= 0 && boneIdx < (int)m_boneHasRb.size() &&
           m_boneHasRb[boneIdx];
  }

  void setFixedTimestep(float t) { m_config.fixedTimestep = t; }
  void setMaxSubSteps(int n) { m_config.maxSubSteps = n; }
  void setSolverIterations(int n) {
    if (m_dynamicsWorld)
      m_dynamicsWorld->getSolverInfo().m_numIterations = n;
    m_config.solverIterations = n;
  }
  void setSpringDamping(float d) { m_config.springDamping = d; }
  void cascadeChildTransforms(model::Model &model, int boneIdx);

  void cascadeAll(model::Model &model, model::BoneNode *bone);

  void getActiveBoneIndices(std::vector<int> &outBoneIndices) const;

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
  std::vector<int> m_activeRigidBodyIndices;
  std::vector<std::unique_ptr<btTypedConstraint>> m_joints;
  Config m_config;
  bool m_created = false;
  std::vector<bool> m_boneHasRb;
};

} // namespace mmbp::physics

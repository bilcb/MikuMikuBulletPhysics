#pragma once
#include "PhysicsPipeline.h"
#include "core/anim/evaluator.h"
#include "core/ik/solver.h"
#include "core/physics/world.h"
#include "core/pmx/types.h"
#include "core/vmd/types.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mmbp::model {
class Model;
class BoneNode;
} // namespace mmbp::model

namespace mmbp::bridge {

class Engine {
public:
  Engine();
  ~Engine();

  bool loadPMX(const uint8_t *data, int size);
  bool loadPMXFile(const char *path);
  bool loadVMD(const uint8_t *data, int size);
  bool loadVMDFile(const char *path);
  bool mergeVMD(const uint8_t *data, int size);
  bool loadVPDFile(const char *path);
  bool buildPhysics();
  int step(float frame, float delta, float *outLocs, float *outQuats,
           int boneCount);

  void setPhysicsConfig(const physics::Config &config);
  const physics::Config &getPhysicsConfig() const { return m_physicsConfig; }

  void setConfigScale(float s) { m_physicsConfig.scale = s; }
  void setConfigGravity(float x, float y, float z) {
    m_physicsConfig.gravity = btVector3(x, y, z);
    if (m_world)
      m_world->getWorld()->setGravity(m_physicsConfig.gravity);
  }
  void setConfigSolverIters(int n) {
    m_physicsConfig.solverIterations = n;
    if (m_world)
      m_world->setSolverIterations(n);
  }
  void setConfigFixedTimestep(float t) {
    m_physicsConfig.fixedTimestep = t;
    if (m_world)
      m_world->setFixedTimestep(t);
  }
  void setConfigMaxSubsteps(int n) {
    m_physicsConfig.maxSubSteps = n;
    if (m_world)
      m_world->setMaxSubSteps(n);
  }
    void setConfigSpringDamping(float d) { m_physicsConfig.springDamping = d; if (m_world) m_world->setSpringDamping(d); }

  void setConfigSpringDampingLinear(int axis, float d) {
    if (axis >= 0 && axis < 3)
      m_physicsConfig.springDampingLinear[axis] = d;
  }
  void setConfigSpringDampingAngular(int axis, float d) {
    if (axis >= 0 && axis < 3)
      m_physicsConfig.springDampingAngular[axis] = d;
  }
  void setConfigSleepDeactivation(bool enable) {
    m_physicsConfig.enableSleepDeactivation = enable;
  }

  void setIKMaxStepAngle(float angle) {
    auto *cfg = const_cast<ik::IkConfig *>(ik::ikConfig());
    cfg->maxStepAngle = angle;
  }
  void setBoneRestInfo(const float *mat3Array, int count);

  int getBoneCount() const;
  int getBoneName(int idx, char *out, int maxLen) const;
  int getRigidBodyCount() const;
  int getJointCount() const;
  const char *getLastError() const;

  enum class ErrorCode : int {
    None = 0,
    FileOpenFailed = 1,
    FileReadFailed = 2,
    PMXParseError = 3,
    VMDParseError = 4,
    VPDParseError = 5,
    ModelLoadFailed = 6,
    PhysicsBuildFailed = 7,
    InvalidArgument = 8,
    NotInitialized = 9,
  };
  ErrorCode getLastErrorCode() const { return m_lastErrorCode; }
  void setLastError(const char *msg) {
    m_lastError = msg ? msg : "";
    m_lastErrorCode = ErrorCode::None;
  }
  void setLastError(ErrorCode code, const char *msg) {
    m_lastError = msg ? msg : "";
    m_lastErrorCode = code;
  }

  const std::vector<std::string> &getWarnings() const {
    return m_vmdData.warnings;
  }
  const model::BoneNode *getModelBone(int idx) const;
  physics::RigidBody *getRigidBody(int idx);
  physics::World *getPhysicsWorld();

  void evalAnimation(float frame);
  void solveIK();
  void stepPhysics(float frame, float delta);
  void syncPhysics(float frame, float delta, int frameCount = 30);
  void getBoneWorldMat4(int idx, float *outMat4) const;
  int getIKCount() const;
  void setIKLoopCount(int boneIdx, int loopCount);
  int getIKLoopCount(int boneIdx) const;
  void getSolverInfo(float *out) const;

  int resetToAnimation(float frame, float *outLocs, float *outQuats,
                       int boneCount);

  int evalOnly(float frame, float *outLocs, float *outQuats, int boneCount);

private:
  std::unique_ptr<model::Model> m_model;
  std::unique_ptr<physics::World> m_world;
  pmx::PMXData m_pmxData;
  vmd::VMDData m_vmdData;
  std::vector<btMatrix3x3> m_restMatrices;
  physics::Config m_physicsConfig;
  bool m_modelLoaded = false;
  bool m_physicsBuilt = false;
  PhysicsPipeline m_pipeline;
  anim::AnimationState m_animState;
  mutable std::string m_lastError;
  ErrorCode m_lastErrorCode = ErrorCode::None;
};

} // namespace mmbp::bridge

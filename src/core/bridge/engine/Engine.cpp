#include "Engine.h"
#include "../adapter/BlenderAdapter.h"
#include "core/anim/evaluator.h"
#include "core/ik/solver.h"
#include "core/model/model.h"
#include "core/node/transform.h"
#include "core/physics/world.h"
#include "core/pmx/parser.h"
#include "core/util/encoding.h"
#include "core/vmd/parser.h"
#include "core/vpd/parser.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace mmbp::bridge {

Engine::Engine() {}

Engine::~Engine() = default;

bool Engine::loadPMX(const uint8_t *data, int size) {
  mmbp::util::resetEncodingCache();
  auto result = pmx::parse_memory(data, size);
  if (!result.ok()) {
    m_lastError =
        result.error().message ? result.error().message : "PMX parse error";
    m_lastErrorCode = ErrorCode::PMXParseError;
    return false;
  }
  m_pmxData = std::move(result).m_value;

  m_model = std::make_unique<model::Model>();
  if (!m_model->load(m_pmxData)) {
    m_lastError = "Model load failed";
    m_lastErrorCode = ErrorCode::ModelLoadFailed;
    return false;
  }
  node::updateAllBoneTransforms(*m_model);
  m_pipeline.updatePostPhysicsFlag(*m_model);
  m_modelLoaded = true;
  return true;
}

bool Engine::loadVMD(const uint8_t *data, int size) {
  auto result = vmd::parse_memory(data, size);
  if (!result.ok()) {
    m_lastError = result.error ? result.error : "VMD parse error";
    m_lastErrorCode = ErrorCode::VMDParseError;
    return false;
  }
  m_vmdData = result.data;
  m_animState.invalidate();
  return true;
}

bool Engine::mergeVMD(const uint8_t *data, int size) {
  auto result = vmd::parse_memory(data, size);
  if (!result.ok()) {
    m_lastError = result.error ? result.error : "VMD parse error";
    m_lastErrorCode = ErrorCode::VMDParseError;
    return false;
  }

  m_vmdData.keyframes.insert(m_vmdData.keyframes.end(),
                             result.data.keyframes.begin(),
                             result.data.keyframes.end());
  m_vmdData.morphKeys.insert(m_vmdData.morphKeys.end(),
                             result.data.morphKeys.begin(),
                             result.data.morphKeys.end());
  m_vmdData.ikKeys.insert(m_vmdData.ikKeys.end(), result.data.ikKeys.begin(),
                          result.data.ikKeys.end());

  m_vmdData.warnings.insert(m_vmdData.warnings.end(),
                            result.data.warnings.begin(),
                            result.data.warnings.end());

  std::sort(m_vmdData.keyframes.begin(), m_vmdData.keyframes.end(),
            [](const vmd::VMDKeyframe &a, const vmd::VMDKeyframe &b) {
              if (a.frame != b.frame)
                return a.frame < b.frame;
              return a.boneName < b.boneName;
            });
  std::sort(m_vmdData.morphKeys.begin(), m_vmdData.morphKeys.end(),
            [](const vmd::MorphKeyframe &a, const vmd::MorphKeyframe &b) {
              if (a.frame != b.frame)
                return a.frame < b.frame;
              return a.name < b.name;
            });
  std::sort(m_vmdData.ikKeys.begin(), m_vmdData.ikKeys.end(),
            [](const vmd::IKKeyframe &a, const vmd::IKKeyframe &b) {
              return a.frame < b.frame;
            });

  {
    auto it =
        std::unique(m_vmdData.keyframes.rbegin(), m_vmdData.keyframes.rend(),
                    [](const vmd::VMDKeyframe &a, const vmd::VMDKeyframe &b) {
                      return a.frame == b.frame && a.boneName == b.boneName;
                    });
    m_vmdData.keyframes.erase(m_vmdData.keyframes.begin(), it.base());
  }
  {
    auto it = std::unique(
        m_vmdData.morphKeys.rbegin(), m_vmdData.morphKeys.rend(),
        [](const vmd::MorphKeyframe &a, const vmd::MorphKeyframe &b) {
          return a.frame == b.frame && a.name == b.name;
        });
    m_vmdData.morphKeys.erase(m_vmdData.morphKeys.begin(), it.base());
  }
  {
    auto it =
        std::unique(m_vmdData.ikKeys.rbegin(), m_vmdData.ikKeys.rend(),
                    [](const vmd::IKKeyframe &a, const vmd::IKKeyframe &b) {
                      return a.frame == b.frame;
                    });
    m_vmdData.ikKeys.erase(m_vmdData.ikKeys.begin(), it.base());
  }

  anim::groupKeyframesByBone(m_vmdData);
  anim::groupMorphKeyframes(m_vmdData);

  m_animState.invalidate();
  return true;
}

bool Engine::loadPMXFile(const char *path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    m_lastError = "Cannot open PMX file";
    m_lastErrorCode = ErrorCode::FileOpenFailed;
    return false;
  }
  auto size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buffer(static_cast<size_t>(size));
  if (!file.read(reinterpret_cast<char *>(buffer.data()), size)) {
    m_lastError = "Failed to read PMX file";
    m_lastErrorCode = ErrorCode::FileReadFailed;
    return false;
  }
  return loadPMX(buffer.data(), static_cast<int>(size));
}

bool Engine::loadVMDFile(const char *path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    m_lastError = "Cannot open VMD file";
    m_lastErrorCode = ErrorCode::FileOpenFailed;
    return false;
  }
  auto size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buffer(static_cast<size_t>(size));
  if (!file.read(reinterpret_cast<char *>(buffer.data()), size)) {
    m_lastError = "Failed to read VMD file";
    m_lastErrorCode = ErrorCode::FileReadFailed;
    return false;
  }
  return loadVMD(buffer.data(), static_cast<int>(size));
}

bool Engine::loadVPDFile(const char *path) {
  auto result = vpd::parse_file(path);
  if (!result.ok()) {
    m_lastError = result.error ? result.error : "VPD parse error";
    m_lastErrorCode = ErrorCode::VPDParseError;
    return false;
  }

  if (!m_modelLoaded || !m_model) {
    m_lastError = "No model loaded";
    m_lastErrorCode = ErrorCode::NotInitialized;
    return false;
  }

  m_model->resetAnimation();

  for (const auto &pb : result.data.bones) {
    auto *bone = m_model->getBone(pb.name);
    if (!bone)
      continue;

    bone->animTranslate =
        btVector3(pb.position[0], pb.position[1], pb.position[2]);
    bone->animRotate = btQuaternion(pb.rotation[1], pb.rotation[2],
                                    pb.rotation[3], pb.rotation[0]);
  }

  for (const auto &pm : result.data.morphs) {
    int idx = m_model->findMorphIndex(pm.name);
    if (idx < 0)
      continue;
    float *wp = m_model->getMorphWeightPtr(idx);
    if (wp)
      *wp = pm.weight;
  }

  for (size_t i = 0; i < m_model->boneCount(); i++) {
    auto *bone = m_model->getBone(i);
    if (bone && bone->isIK)
      bone->ikEnabled = true;
  }

  node::updateAllBoneTransforms(*m_model);

  for (size_t i = 0; i < m_model->boneCount(); i++) {
    auto *bone = m_model->getBone(i);
    if (bone && bone->isIK && bone->ikEnabled)
      ik::solve(*m_model, *bone);
  }

  node::updateAllBoneTransforms(*m_model);
  return true;
}

bool Engine::buildPhysics() {
  if (!m_modelLoaded) {
    m_lastError = "No model loaded";
    m_lastErrorCode = ErrorCode::NotInitialized;
    return false;
  }

  m_world = std::make_unique<physics::World>();
  if (!m_world->create(m_physicsConfig)) {
    m_lastError = "Physics world creation failed";
    m_lastErrorCode = ErrorCode::PhysicsBuildFailed;
    return false;
  }

  if (m_model)
    node::updateAllBoneTransforms(*m_model);

  for (size_t i = 0; i < m_pmxData.rigidBodies.size(); i++) {
    m_world->addRigidBody(
        m_pmxData.rigidBodies[i], m_pmxData.rigidBodies[i].boneIdx, 1.0f,
        m_model ? m_model->getBone(m_pmxData.rigidBodies[i].boneIdx) : nullptr);
  }

  for (size_t i = 0; i < m_pmxData.joints.size(); i++) {
    const auto &jd = m_pmxData.joints[i];
    auto *rbA = m_world->getRigidBody(jd.rigidBodyAIdx);
    auto *rbB = m_world->getRigidBody(jd.rigidBodyBIdx);
    m_world->addJoint(jd, rbA, rbB);
  }

  for (int i = 0; i < m_world->rigidBodyCount(); i++) {
    if (auto *rb = m_world->getRigidBody(i)) {
      if (rb->body) {
        rb->body->setSleepingThresholds(0.01f, 0.1f);
      }
    }
  }

  m_physicsBuilt = true;
  return true;
}

int Engine::step(float frame, float delta, float *outLocs, float *outQuats,
                 int boneCount) {
  if (!m_modelLoaded || !m_model)
    return 0;
  physics::World *worldPtr =
      (m_physicsBuilt && m_world) ? m_world.get() : nullptr;
  auto result = m_pipeline.step(
      m_animState, *m_model, worldPtr, m_vmdData, frame, delta, m_restMatrices,
      m_physicsConfig.scale, outLocs, outQuats, boneCount);
  return result.boneCount;
}

void Engine::setBoneRestInfo(const float *mat3Array, int count) {
  m_restMatrices.resize(count);
  for (int i = 0; i < count; i++) {
    m_restMatrices[i] = btMatrix3x3(
        mat3Array[i * 9 + 0], mat3Array[i * 9 + 1], mat3Array[i * 9 + 2],
        mat3Array[i * 9 + 3], mat3Array[i * 9 + 4], mat3Array[i * 9 + 5],
        mat3Array[i * 9 + 6], mat3Array[i * 9 + 7], mat3Array[i * 9 + 8]);
  }
}

void Engine::setPhysicsConfig(const physics::Config &config) {
  m_physicsConfig = config;
  if (m_world) {
    m_world->getWorld()->setGravity(m_physicsConfig.gravity);
  }
}

int Engine::getBoneCount() const {
  return m_model ? (int)m_model->boneCount() : 0;
}

int Engine::getBoneName(int idx, char *out, int maxLen) const {
  if (!m_model || idx < 0 || idx >= (int)m_model->boneCount()) {
    if (maxLen > 0)
      out[0] = 0;
    return 0;
  }
  auto *bone = m_model->getBone(idx);
  if (!bone) {
    if (maxLen > 0)
      out[0] = 0;
    return 0;
  }
  strncpy(out, bone->name.c_str(), maxLen - 1);
  out[maxLen - 1] = 0;
  return 1;
}

int Engine::getRigidBodyCount() const {
  return m_world ? m_world->rigidBodyCount() : 0;
}

int Engine::getJointCount() const {
  return m_world ? m_world->jointCount() : 0;
}

const char *Engine::getLastError() const { return m_lastError.c_str(); }

void Engine::evalAnimation(float frame) {
  if (!m_modelLoaded || !m_model)
    return;
  m_model->resetAnimation();
  anim::evaluate(m_animState, m_vmdData, *m_model, frame);
  anim::evaluateIKKeys(m_animState, m_vmdData, *m_model, frame);
  node::updateAllBoneTransforms(*m_model);
}

void Engine::solveIK() {
  if (!m_modelLoaded || !m_model)
    return;

  for (size_t i = 0; i < m_model->boneCount(); i++) {
    auto *bone = m_model->getBone(i);
    if (bone && bone->isIK && bone->ikEnabled) {
      ik::solve(*m_model, *bone);
    }
  }
  node::updateAllBoneTransforms(*m_model);
}

void Engine::stepPhysics(float frame, float delta) {
  if (!m_modelLoaded || !m_model || !m_physicsBuilt || !m_world)
    return;
  m_world->step(delta);
  m_world->syncBoneTransforms(*m_model);
  for (int i = 0; i < m_world->rigidBodyCount(); i++) {
    auto *rb = m_world->getRigidBody(i);
    if (!rb || rb->boneIdx < 0)
      continue;
    auto *bone = m_model->getBone(rb->boneIdx);
    if (bone) {
      node::updateChildTransforms(*bone, *m_model);
    }
  }
}

void Engine::syncPhysics(float frame, float delta, int frameCount) {
  if (!m_modelLoaded || !m_model || !m_physicsBuilt || !m_world)
    return;
  if (frameCount <= 0)
    frameCount = 1;

  m_model->saveBaseAnimation();

  for (int i = 0; i < frameCount; i++) {
    float weight = float(1 + i) / float(frameCount);

    m_model->loadBaseAnimation();
    anim::evaluate(m_animState, m_vmdData, *m_model, frame, weight);
    anim::evaluateIKKeys(m_animState, m_vmdData, *m_model, frame);
    node::updateAllBoneTransforms(*m_model);

    for (size_t j = 0; j < m_model->boneCount(); j++) {
      auto *bone = m_model->getBone(j);
      if (bone && bone->isIK && bone->ikEnabled) {
        ik::solve(*m_model, *bone);
      }
    }
    node::updateAllBoneTransforms(*m_model);

    m_world->step(delta / float(frameCount));
    m_world->syncBoneTransforms(*m_model);
    for (int j = 0; j < m_world->rigidBodyCount(); j++) {
      auto *rb = m_world->getRigidBody(j);
      if (!rb || rb->boneIdx < 0)
        continue;
      auto *bone = m_model->getBone(rb->boneIdx);
      if (bone) {
        node::updateChildTransforms(*bone, *m_model);
      }
    }
  }
}

void Engine::getBoneWorldMat4(int idx, float *outMat4) const {
  if (!m_model || idx < 0 || idx >= (int)m_model->boneCount()) {
    memset(outMat4, 0, 16 * sizeof(float));
    outMat4[0] = outMat4[5] = outMat4[10] = outMat4[15] = 1.0f;
    return;
  }
  auto *bone = m_model->getBone(idx);
  if (!bone) {
    memset(outMat4, 0, 16 * sizeof(float));
    outMat4[0] = outMat4[5] = outMat4[10] = outMat4[15] = 1.0f;
    return;
  }
  bone->global.getOpenGLMatrix(outMat4);
}

int Engine::getIKCount() const {
  if (!m_model)
    return 0;
  int count = 0;
  for (size_t i = 0; i < m_model->boneCount(); i++) {
    auto *bone = m_model->getBone(i);
    if (bone && bone->isIK)
      count++;
  }
  return count;
}

const model::BoneNode *Engine::getModelBone(int idx) const {
  return m_model ? m_model->getBone(idx) : nullptr;
}

physics::RigidBody *Engine::getRigidBody(int idx) {
  return m_world ? m_world->getRigidBody(idx) : nullptr;
}

physics::World *Engine::getPhysicsWorld() { return m_world.get(); }

void Engine::setIKLoopCount(int boneIdx, int loopCount) {
  if (!m_model)
    return;
  auto *bone = m_model->getBone(boneIdx);
  if (bone && bone->isIK) {
    bone->ikLoopCount = std::max(1, std::min(1000, loopCount));
  }
}

int Engine::getIKLoopCount(int boneIdx) const {
  if (!m_model)
    return 0;
  auto *bone = m_model->getBone(boneIdx);
  return (bone && bone->isIK) ? bone->ikLoopCount : 0;
}

void Engine::getSolverInfo(float *out) const {
  memset(out, 0, 18 * sizeof(float));
  if (m_world) {
    auto *world = m_world->getWorld();
    if (world) {
      auto &si = world->getSolverInfo();
      out[1] = (float)si.m_numIterations;
      out[2] = si.m_erp;
      out[3] = si.m_erp2;
      out[4] = si.m_globalCfm;
      out[5] = si.m_splitImpulse ? 1.0f : 0.0f;
      out[6] = si.m_splitImpulsePenetrationThreshold;
      out[10] = world->getDispatchInfo().m_allowedCcdPenetration;
      out[15] = world->getDispatchInfo().m_allowedCcdPenetration;
      out[16] = (float)si.m_solverMode;
      auto g = world->getGravity();
      out[12] = g.x();
      out[13] = g.y();
      out[14] = g.z();
    }
  } else {
    out[1] = (float)m_physicsConfig.solverIterations;
    out[2] = 0.2f;
    out[3] = 0.2f;
    out[5] = 1.0f;
    out[6] = -0.04f;
    out[10] = 0.04f;
    out[15] = 0.04f;
    out[16] = 260.0f;
    out[12] = m_physicsConfig.gravity.x();
    out[13] = m_physicsConfig.gravity.y();
    out[14] = m_physicsConfig.gravity.z();
  }
  out[17] = (float)m_physicsConfig.maxSubSteps;
}

int Engine::resetToAnimation(float frame, float *outLocs, float *outQuats,
                             int boneCount) {
  if (!m_modelLoaded || !m_model)
    return 0;
  physics::World *worldPtr =
      (m_physicsBuilt && m_world) ? m_world.get() : nullptr;
  auto result = m_pipeline.resetToAnimation(
      m_animState, *m_model, worldPtr, m_vmdData, frame, m_restMatrices,
      m_physicsConfig.scale, outLocs, outQuats, boneCount);
  return result.boneCount;
}

int Engine::evalOnly(float frame, float *outLocs, float *outQuats,
                     int boneCount) {
  if (!m_modelLoaded || !m_model)
    return 0;
  auto result = m_pipeline.evalOnly(m_animState, *m_model, m_vmdData, frame,
                                    m_restMatrices, m_physicsConfig.scale,
                                    outLocs, outQuats, boneCount);
  return result.boneCount;
}

} // namespace mmbp::bridge

#include "Engine.h"
#include "core/model/model.h"
#include "core/physics/world.h"
#include "core/pmx/parser.h"
#include "core/vmd/parser.h"
#include "core/anim/evaluator.h"
#include "core/ik/solver.h"
#include "core/node/transform.h"
#include "../adapter/BlenderAdapter.h"
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

namespace mmp::bridge {

Engine::Engine() {
    BlenderAdapter::initialize();
}

Engine::~Engine() = default;

static void updateAllBoneTransforms(model::Model& model);

bool Engine::loadPMX(const uint8_t* data, int size) {
    auto result = pmx::parse_memory(data, size);
    if (!result.ok()) {
        m_lastError = result.error().message ? result.error().message : "PMX parse error";
        return false;
    }
    m_pmxData = std::move(result).m_value;

    m_model = std::make_unique<model::Model>();
    if (!m_model->load(m_pmxData)) {
        m_lastError = "Model load failed";
        return false;
    }
    // Compute initial global transforms and inverse init matrices
    updateAllBoneTransforms(*m_model);
    for (size_t i = 0; i < m_model->boneCount(); i++) {
        auto* bone = m_model->getBone(i);
        if (bone) bone->inverseInit = bone->global.inverse();
    }

    m_modelLoaded = true;
    return true;
}

bool Engine::loadVMD(const uint8_t* data, int size) {
    auto result = vmd::parse_memory(data, size);
    if (!result.ok()) {
        m_lastError = result.error ? result.error : "VMD parse error";
        return false;
    }
    m_vmdData = result.data;
    return true;
}

bool Engine::loadPMXFile(const char* path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        m_lastError = "Cannot open PMX file";
        return false;
    }
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        m_lastError = "Failed to read PMX file";
        return false;
    }
    return loadPMX(buffer.data(), static_cast<int>(size));
}

bool Engine::loadVMDFile(const char* path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        m_lastError = "Cannot open VMD file";
        return false;
    }
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        m_lastError = "Failed to read VMD file";
        return false;
    }
    return loadVMD(buffer.data(), static_cast<int>(size));
}

bool Engine::buildPhysics() {
    if (!m_modelLoaded) {
        m_lastError = "No model loaded";
        return false;
    }

    m_world = std::make_unique<physics::World>();
    if (!m_world->create(m_physicsConfig)) {
        m_lastError = "Physics world creation failed";
        return false;
    }

    // Compute bone global transforms for offset computation
    if (m_model) updateAllBoneTransforms(*m_model);

    for (size_t i = 0; i < m_pmxData.rigidBodies.size(); i++) {
        m_world->addRigidBody(m_pmxData.rigidBodies[i], m_pmxData.rigidBodies[i].boneIdx, 1.0f,
            m_model ? m_model->getBone(m_pmxData.rigidBodies[i].boneIdx) : nullptr);
    }

    for (size_t i = 0; i < m_pmxData.joints.size(); i++) {
        const auto& jd = m_pmxData.joints[i];
        auto* rbA = m_world->getRigidBody(jd.rigidBodyAIdx);
        auto* rbB = m_world->getRigidBody(jd.rigidBodyBIdx);
        m_world->addJoint(jd, rbA, rbB);
    }

    for (int i = 0; i < m_world->rigidBodyCount(); i++) {
        m_world->setActivation(i, false);
    }

    m_physicsBuilt = true;
    return true;
}

static void updateAllBoneTransforms(model::Model& model) {
    node::updateAppendTransforms(model);
    size_t n = model.boneCount();
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        if (bone) node::updateLocalTransform(*bone);
    }
    for (size_t i = 0; i < n; i++) {
        auto* bone = model.getBone(i);
        if (bone && bone->parentIdx < 0) {
            node::updateGlobalTransform(*bone, model);
            node::updateChildTransforms(*bone, model);
        }
    }
}

int Engine::step(float frame, float delta, float* outLocs, float* outQuats, int boneCount) {
    if (!m_modelLoaded || !m_model) return 0;
    physics::World* worldPtr = (m_physicsBuilt && m_world) ? m_world.get() : nullptr;
    auto result = m_pipeline.step(
        *m_model, worldPtr, m_vmdData, frame, delta,
        m_restMatrices, m_restPosOffsets, m_physicsConfig.scale,
        outLocs, outQuats, boneCount
    );
    return result.boneCount;
}

void Engine::setBoneRestInfo(const float* mat3Array, int count) {
    m_restMatrices.resize(count);
    for (int i = 0; i < count; i++) {
        m_restMatrices[i] = btMatrix3x3(
            mat3Array[i * 9 + 0], mat3Array[i * 9 + 1], mat3Array[i * 9 + 2],
            mat3Array[i * 9 + 3], mat3Array[i * 9 + 4], mat3Array[i * 9 + 5],
            mat3Array[i * 9 + 6], mat3Array[i * 9 + 7], mat3Array[i * 9 + 8]);
    }
}

void Engine::setBoneRestPositions(const float* offsetArray, int count) {
    m_restPosOffsets.resize(count);
    for (int i = 0; i < count; i++) {
        m_restPosOffsets[i] = btVector3(offsetArray[i * 3 + 0], offsetArray[i * 3 + 1], offsetArray[i * 3 + 2]);
    }
}

void Engine::setPhysicsConfig(const physics::Config& config) {
    m_physicsConfig = config;
    if (m_world) {
        m_world->getWorld()->setGravity(m_physicsConfig.gravity);
    }
}

int Engine::getBoneCount() const {
    return m_model ? (int)m_model->boneCount() : 0;
}

int Engine::getBoneName(int idx, char* out, int maxLen) const {
    if (!m_model || idx < 0 || idx >= (int)m_model->boneCount()) {
        if (maxLen > 0) out[0] = 0;
        return 0;
    }
    auto* bone = m_model->getBone(idx);
    if (!bone) {
        if (maxLen > 0) out[0] = 0;
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

const char* Engine::getLastError() const {
    return m_lastError.c_str();
}

void Engine::evalAnimation(float frame) {
    if (!m_modelLoaded || !m_model) return;
    m_model->resetAnimation();
    anim::evaluate(m_vmdData, *m_model, frame);
    anim::evaluateIKKeys(m_vmdData, *m_model, frame);
    updateAllBoneTransforms(*m_model);
}

void Engine::solveIK() {
    if (!m_modelLoaded || !m_model) return;
    for (size_t i = 0; i < m_model->boneCount(); i++) {
        auto* bone = m_model->getBone(i);
        if (bone && bone->isIK && bone->ikEnabled) {
            ik::solve(*m_model, *bone);
        }
    }
    updateAllBoneTransforms(*m_model);
}

void Engine::stepPhysics(float frame, float delta) {
    if (!m_modelLoaded || !m_model || !m_physicsBuilt || !m_world) return;
    m_world->step(delta);
    m_world->syncBoneTransforms(*m_model);
    for (int i = 0; i < m_world->rigidBodyCount(); i++) {
        auto* rb = m_world->getRigidBody(i);
        if (!rb || rb->boneIdx < 0) continue;
        auto* bone = m_model->getBone(rb->boneIdx);
        if (bone) {
            node::updateChildTransforms(*bone, *m_model);
        }
    }
}

void Engine::getBoneWorldMat4(int idx, float* outMat4) const {
    if (!m_model || idx < 0 || idx >= (int)m_model->boneCount()) {
        memset(outMat4, 0, 16 * sizeof(float));
        outMat4[0] = outMat4[5] = outMat4[10] = outMat4[15] = 1.0f;
        return;
    }
    auto* bone = m_model->getBone(idx);
    if (!bone) {
        memset(outMat4, 0, 16 * sizeof(float));
        outMat4[0] = outMat4[5] = outMat4[10] = outMat4[15] = 1.0f;
        return;
    }
    bone->global.getOpenGLMatrix(outMat4);
}

int Engine::getIKCount() const {
    if (!m_model) return 0;
    int count = 0;
    for (size_t i = 0; i < m_model->boneCount(); i++) {
        auto* bone = m_model->getBone(i);
        if (bone && bone->isIK) count++;
    }
    return count;
}

const model::BoneNode* Engine::getModelBone(int idx) const {
    return m_model ? m_model->getBone(idx) : nullptr;
}

physics::RigidBody* Engine::getRigidBody(int idx) {
    return m_world ? m_world->getRigidBody(idx) : nullptr;
}

void Engine::getSolverInfo(float* out) const {
    memset(out, 0, 18 * sizeof(float));
    if (m_world) {
        auto* world = m_world->getWorld();
        if (world) {
            auto& si = world->getSolverInfo();
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
            out[12] = g.x(); out[13] = g.y(); out[14] = g.z();
        }
    } else {
        out[1] = (float)m_physicsConfig.solverIterations;
        out[2] = 0.2f; out[3] = 0.2f;
        out[5] = 1.0f; out[6] = -0.04f;
        out[10] = 0.04f; out[15] = 0.04f;
        out[16] = 260.0f;
        out[12] = m_physicsConfig.gravity.x(); out[13] = m_physicsConfig.gravity.y(); out[14] = m_physicsConfig.gravity.z();
    }
    out[17] = (float)m_physicsConfig.maxSubSteps;
}

} // namespace mmp::bridge

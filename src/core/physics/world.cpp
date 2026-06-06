#include "world.h"
#include "core/math/converter.h"
#include "core/node/transform.h"
#include "core/util/logger.h"
#include <algorithm>
#include <cstring>

namespace mmbp::physics {

void MMDMotionState::resetFromNode(model::BoneNode *node,
                                   const btTransform &offset, float scale,
                                   btTransform &outTransform) {
  if (node) {
    btTransform boneGlobal = node->global;
    boneGlobal.setOrigin(mmbp::math::applyScale(boneGlobal.getOrigin(), scale));
    outTransform = boneGlobal * offset;
  }
}

KinematicMotionState::KinematicMotionState(model::BoneNode *node,
                                           const btTransform &offset,
                                           float scale)
    : m_node(node), m_offset(offset), m_scale(scale) {}

void KinematicMotionState::getWorldTransform(
    btTransform &worldTransform) const {
  if (m_node) {
    btTransform boneGlobal = m_node->global;
    boneGlobal.setOrigin(
        mmbp::math::applyScale(boneGlobal.getOrigin(), m_scale));
    worldTransform = boneGlobal * m_offset;
  } else {
    worldTransform = m_offset;
  }
}

void KinematicMotionState::setWorldTransform(const btTransform &) {}

DynamicMotionState::DynamicMotionState(model::BoneNode *node,
                                       const btTransform &offset, float scale)
    : m_node(node), m_offset(offset), m_scale(scale) {
  m_invOffset = m_offset.inverse();

  if (!m_node) {
    m_transform = m_offset;
  }
  reset();
}

void DynamicMotionState::getWorldTransform(btTransform &worldTransform) const {
  worldTransform = m_transform;
}

void DynamicMotionState::setWorldTransform(const btTransform &worldTransform) {
  m_transform = worldTransform;
}

void DynamicMotionState::reset() {
  resetFromNode(m_node, m_offset, m_scale, m_transform);
}

void DynamicMotionState::reflectGlobalTransform() {
  if (!m_node)
    return;
  m_node->global = m_transform * m_invOffset;
  m_node->global.setOrigin(
      mmbp::math::removeScale(m_node->global.getOrigin(), m_scale));
}

DynamicBoneMergeMotionState::DynamicBoneMergeMotionState(
    model::BoneNode *node, const btTransform &offset, float scale)
    : DynamicMotionState(node, offset, scale) {}

void DynamicBoneMergeMotionState::reflectGlobalTransform() {
  if (!m_node)
    return;

  btTransform btGlobal = m_transform * m_invOffset;
  btGlobal.setOrigin(mmbp::math::removeScale(btGlobal.getOrigin(), m_scale));
  btVector3 origPos = m_node->global.getOrigin();
  btGlobal.setOrigin(origPos);
  m_node->global = btGlobal;
}

World::~World() { destroy(); }

bool World::create(const Config &cfg) {
  m_config = cfg;

  try {
    m_collisionCfg = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher =
        std::make_unique<btCollisionDispatcher>(m_collisionCfg.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_dynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(),
        m_collisionCfg.get());

    m_dynamicsWorld->setGravity(m_config.gravity);
    m_dynamicsWorld->getSolverInfo().m_numIterations =
        m_config.solverIterations;

    auto &si = m_dynamicsWorld->getSolverInfo();
    si.m_erp = 0.6f;
    si.m_erp2 = 0.4f;
    si.m_globalCfm = 0.0001f;
    si.m_splitImpulsePenetrationThreshold = -0.02f;
    si.m_warmstartingFactor = 0.95f;
    si.m_damping = 0.8f;
    si.m_solverMode = SOLVER_USE_WARMSTARTING | SOLVER_SIMD |
                       SOLVER_CACHE_FRIENDLY |
                       SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION;
    m_dynamicsWorld->setForceUpdateAllAabbs(false);

    m_groundShape = std::make_unique<btStaticPlaneShape>(btVector3(0, 1, 0), 0);
    m_groundMotionState = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundCI(
        0, m_groundMotionState.get(), m_groundShape.get());
    m_groundBody = std::make_unique<btRigidBody>(groundCI);
    m_dynamicsWorld->addRigidBody(m_groundBody.get());

    m_filterCallback = std::make_unique<MMDFilterCallback>();
    m_filterCallback->m_nonFilterProxies.push_back(
        m_groundBody->getBroadphaseProxy());
    m_dynamicsWorld->getPairCache()->setOverlapFilterCallback(
        m_filterCallback.get());
  } catch (const std::exception &e) {
    g_logger.error("[MMBP] World::create failed: %s", e.what());
    destroy();
    return false;
  }

  m_created = true;
  return true;
}

void World::destroy() {

#ifndef NDEBUG
  size_t preBodies = m_rigidBodies.size();
  size_t preJoints = m_joints.size();
#endif
  for (int i = (int)m_joints.size() - 1; i >= 0; i--) {
    if (m_dynamicsWorld)
      m_dynamicsWorld->removeConstraint(m_joints[i].get());
  }
  m_joints.clear();

  for (auto &rb : m_rigidBodies) {
    if (rb.activeMotionState)
      rb.activeMotionState->clearNode();
    if (rb.kinematicMotionState)
      rb.kinematicMotionState->clearNode();
  }

  for (auto &rb : m_rigidBodies) {
    if (m_dynamicsWorld && rb.body) {
      m_dynamicsWorld->removeRigidBody(rb.body.get());
    }
  }
  m_rigidBodies.clear();
  m_activeRigidBodyIndices.clear();

  if (m_dynamicsWorld && m_groundBody) {
    m_dynamicsWorld->removeRigidBody(m_groundBody.get());
  }
  m_groundBody.reset();
  m_groundShape.reset();
  m_groundMotionState.reset();
  m_filterCallback.reset();

  m_dynamicsWorld.reset();
  m_solver.reset();
  m_broadphase.reset();
  m_dispatcher.reset();
  m_collisionCfg.reset();
  m_created = false;
}

static std::unique_ptr<btCollisionShape>
createShape(const pmx::PMXRigidBody &rbData, float scale) {
  const float s = scale;
  switch (rbData.shapeType) {
  case 0: {
    float radius = rbData.size.x() * s;
    return std::make_unique<btSphereShape>(radius);
  }
  case 1: {
    btVector3 half(rbData.size.x() * s, rbData.size.y() * s,
                   rbData.size.z() * s);
    return std::make_unique<btBoxShape>(half);
  }
  case 2: {
    float radius = rbData.size.x() * s;
    float height = rbData.size.y() * s;
    return std::make_unique<btCapsuleShape>(radius, height);
  }
  default: {
    btVector3 half(rbData.size.x() * s, rbData.size.y() * s,
                   rbData.size.z() * s);
    return std::make_unique<btBoxShape>(half);
  }
  }
}

int World::addRigidBody(const pmx::PMXRigidBody &rbData, int boneIdx,
                        float scale, model::BoneNode *bone) {
  RigidBody rb;
  rb.boneIdx = boneIdx;
  rb.group = rbData.group;
  rb.groupMask = rbData.groupMask;
  rb.mode = rbData.mode;
  rb.shape = createShape(rbData, scale);

  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(mmbp::math::applyScale(rbData.position, scale));
  startTransform.setRotation(rbData.rotation);

  if (bone) {
    btTransform boneWorldPhys = bone->global;
    boneWorldPhys.setOrigin(
        mmbp::math::applyScale(boneWorldPhys.getOrigin(), scale));
    rb.offsetMat = boneWorldPhys.inverse() * startTransform;
  }

  if (bone) {

    if (rbData.mode == 0) {
      rb.kinematicMotionState =
          std::make_unique<KinematicMotionState>(bone, rb.offsetMat, scale);
    } else if (rbData.mode == 1) {
      rb.activeMotionState =
          std::make_unique<DynamicMotionState>(bone, rb.offsetMat, scale);
      rb.kinematicMotionState =
          std::make_unique<KinematicMotionState>(bone, rb.offsetMat, scale);
    } else if (rbData.mode == 2) {
      rb.activeMotionState = std::make_unique<DynamicBoneMergeMotionState>(
          bone, rb.offsetMat, scale);
      rb.kinematicMotionState =
          std::make_unique<KinematicMotionState>(bone, rb.offsetMat, scale);
    }
  } else {

    if (rbData.mode == 0) {
      rb.kinematicMotionState =
          std::make_unique<DefaultMotionState>(rb.offsetMat);
    } else {
      rb.activeMotionState = std::make_unique<DefaultMotionState>(rb.offsetMat);
      rb.kinematicMotionState =
          std::make_unique<DefaultMotionState>(rb.offsetMat);
    }
  }

  btMotionState *motionState = nullptr;
  if (rbData.mode == 0) {
    motionState = rb.kinematicMotionState.get();
  } else {
    motionState = rb.activeMotionState.get();
  }

  bool isStatic = (rbData.mode == 0);
  btScalar mass = isStatic ? 0.0f : rbData.mass;
  btVector3 localInertia(0, 0, 0);
  if (!isStatic) {
    rb.shape->calculateLocalInertia(mass, localInertia);
  }

  btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, rb.shape.get(),
                                              localInertia);
  ci.m_linearDamping = rbData.linearDamping;
  ci.m_angularDamping = rbData.angularDamping;
  ci.m_restitution = rbData.restitution;
  ci.m_friction = rbData.friction;
  ci.m_additionalDamping = true;

  rb.body = std::make_unique<btRigidBody>(ci);

  if (m_config.enableSleepDeactivation) {
    rb.body->setSleepingThresholds(0.01f, 0.001745f);
  } else {
    rb.body->setActivationState(DISABLE_DEACTIVATION);
  }
  rb.body->setUserPointer(&rb);

  if (isStatic) {
    rb.body->setCollisionFlags(rb.body->getCollisionFlags() |
                               btCollisionObject::CF_KINEMATIC_OBJECT);
  }

  if (m_dynamicsWorld)
    m_dynamicsWorld->addRigidBody(rb.body.get(), 1 << (rbData.group & 0xF),
                                  rbData.groupMask);

  if (boneIdx >= (int)m_boneHasRb.size())
    m_boneHasRb.resize(boneIdx + 1, 0);
  if (boneIdx >= 0)
    m_boneHasRb[boneIdx] = 1;

  int idx = (int)m_rigidBodies.size();
  m_rigidBodies.push_back(std::move(rb));

  if (rbData.mode != 0) {
    m_activeRigidBodyIndices.push_back(idx);
  }
  return idx;
}

void World::addJoint(const pmx::PMXJoint &jointData, const RigidBody *rbA,
                     const RigidBody *rbB) {
  if (!rbA || !rbB || !rbA->body || !rbB->body)
    return;

  btTransform frameA, frameB;
  frameA.setIdentity();
  frameB.setIdentity();

  frameA.setOrigin(jointData.position);
  frameB.setOrigin(jointData.position);
  frameA.setRotation(jointData.rotation);
  frameB.setRotation(jointData.rotation);

  frameA = rbA->body->getWorldTransform().inverse() * frameA;
  frameB = rbB->body->getWorldTransform().inverse() * frameB;

  auto constraint = std::make_unique<btGeneric6DofSpringConstraint>(
      *rbA->body, *rbB->body, frameA, frameB, true);

  constraint->setLinearLowerLimit(jointData.linearLowerLimit);
  constraint->setLinearUpperLimit(jointData.linearUpperLimit);
  constraint->setAngularLowerLimit(jointData.angularLowerLimit);
  constraint->setAngularUpperLimit(jointData.angularUpperLimit);

  for (int i = 0; i < 6; i++) {
    constraint->setParam(BT_CONSTRAINT_STOP_ERP, 0.8, i);
    constraint->setParam(BT_CONSTRAINT_STOP_CFM, 0.0001, i);
  }

  {
    float sp[3] = {jointData.springTranslate.x(), jointData.springTranslate.y(),
                   jointData.springTranslate.z()};
    float sr[3] = {jointData.springRotate.x(), jointData.springRotate.y(),
                   jointData.springRotate.z()};
    bool hasSpring = false;
    for (int i = 0; i < 3 && !hasSpring; i++) {
      if (sp[i] != 0.0f || sr[i] != 0.0f)
        hasSpring = true;
    }

    if (hasSpring) {
      const float softEpsilon = std::max(0.0001f, m_config.scale * 0.01f);
      btVector3 linLower = jointData.linearLowerLimit;
      btVector3 linUpper = jointData.linearUpperLimit;
      btVector3 angLower = jointData.angularLowerLimit;
      btVector3 angUpper = jointData.angularUpperLimit;
      for (int i = 0; i < 3; i++) {
        if (std::abs(linUpper[i] - linLower[i]) < 1e-6f) {
          linLower[i] -= softEpsilon;
          linUpper[i] += softEpsilon;
        }
        if (std::abs(angUpper[i] - angLower[i]) < 1e-6f) {
          angLower[i] -= softEpsilon;
          angUpper[i] += softEpsilon;
        }
      }
      constraint->setLinearLowerLimit(linLower);
      constraint->setLinearUpperLimit(linUpper);
      constraint->setAngularLowerLimit(angLower);
      constraint->setAngularUpperLimit(angUpper);
    }

    for (int i = 0; i < 3; i++) {
      if (sp[i] != 0.0f) {
        constraint->enableSpring(i, true);
        constraint->setStiffness(i, sp[i]);
        float damp = m_config.springDampingLinear[i] != 0.0f
                         ? m_config.springDampingLinear[i]
                         : m_config.springDamping;
        constraint->setDamping(i, damp);
      }
      if (sr[i] != 0.0f) {
        constraint->enableSpring(i + 3, true);
        constraint->setStiffness(i + 3, sr[i]);
        float damp = m_config.springDampingAngular[i] != 0.0f
                         ? m_config.springDampingAngular[i]
                         : m_config.springDamping;
        constraint->setDamping(i + 3, damp);
      }
    }
  }

  m_joints.reserve(m_joints.size() + 1);
  if (m_dynamicsWorld)
    m_dynamicsWorld->addConstraint(constraint.get());
  m_joints.push_back(std::move(constraint));
}

void World::step(float delta) {
  if (m_dynamicsWorld) {
    m_dynamicsWorld->stepSimulation(delta, m_config.maxSubSteps,
                                    m_config.fixedTimestep);
  }
}

void World::resetPhysics(model::Model &model) {
  if (!m_dynamicsWorld)
    return;
  auto *cache = m_dynamicsWorld->getPairCache();
  auto *dispatcher = m_dynamicsWorld->getDispatcher();
  for (auto &rb : m_rigidBodies) {
    if (!rb.body || rb.boneIdx < 0)
      continue;
    auto *bone = model.getBone(rb.boneIdx);
    if (!bone)
      continue;
    if (cache) {
      cache->cleanProxyFromPairs(rb.body->getBroadphaseHandle(), dispatcher);
    }
    rb.body->clearForces();
    rb.body->setLinearVelocity(btVector3(0, 0, 0));
    rb.body->setAngularVelocity(btVector3(0, 0, 0));
    if (rb.activeMotionState)
      rb.activeMotionState->reset();
    if (rb.kinematicMotionState)
      rb.kinematicMotionState->reset();
    if (rb.kinematicMotionState) {
      btTransform t;
      rb.kinematicMotionState->getWorldTransform(t);
      rb.body->setCenterOfMassTransform(t);
    }
  }
}

void World::syncBoneTransforms(model::Model &model) {

  for (int idx : m_activeRigidBodyIndices) {
    auto &rb = m_rigidBodies[idx];
    if (!rb.body || rb.boneIdx < 0)
      continue;
    auto *bone = model.getBone(rb.boneIdx);
    if (!bone)
      continue;

    if (rb.activeMotionState && !(rb.body->getCollisionFlags() &
                                  btCollisionObject::CF_KINEMATIC_OBJECT)) {
      btTransform physTransform = rb.body->getWorldTransform();
      rb.activeMotionState->setWorldTransform(physTransform);
      rb.activeMotionState->reflectGlobalTransform();
    }
  }

  size_t n = model.boneCount();
  for (size_t i = 0; i < n; i++) {
    auto *bone = model.getBone(i);
    if (!bone || bone->parentIdx >= 0)
      continue;
    cascadeAll(model, bone);
  }
}

void World::cascadeAll(model::Model &model, model::BoneNode *bone) {
  if (!bone)
    return;
  for (int32_t childIdx : bone->children) {
    auto *child = model.getBone(childIdx);
    if (!child)
      continue;

    if (childIdx < (int)m_boneHasRb.size() && m_boneHasRb[childIdx]) {
      child->global.setOrigin(bone->global * child->local.getOrigin());
      cascadeAll(model, child);
      continue;
    }
    node::updateLocalTransform(*child);
    child->global = bone->global * child->local;
    cascadeAll(model, child);
  }
}

void World::cascadeChildTransforms(model::Model &model, int boneIdx) {
  auto *bone = model.getBone(boneIdx);
  if (!bone)
    return;
  for (int32_t childIdx : bone->children) {
    auto *child = model.getBone(childIdx);
    if (!child)
      continue;
    if (childIdx < (int)m_boneHasRb.size() && m_boneHasRb[childIdx]) {
      cascadeChildTransforms(model, childIdx);
      continue;
    }
    node::updateLocalTransform(*child);
    child->global = bone->global * child->local;
    cascadeChildTransforms(model, childIdx);
  }
}

void World::setActivation(int idx, bool active) {
  auto *rb = getRigidBody(idx);
  if (!rb || !rb->body)
    return;

  if (rb->mode == 0) {
    rb->body->setMotionState(rb->kinematicMotionState.get());
    return;
  }

  if (active) {
    if (rb->activeMotionState) {
      btTransform savedWorld;
      if (rb->body->getCollisionFlags() &
          btCollisionObject::CF_KINEMATIC_OBJECT) {
        rb->kinematicMotionState->getWorldTransform(savedWorld);
      } else {
        savedWorld = rb->body->getWorldTransform();
      }

      rb->activeMotionState->reset();
      rb->body->setMotionState(rb->activeMotionState.get());
      rb->body->setWorldTransform(savedWorld);
      rb->body->setCollisionFlags(rb->body->getCollisionFlags() &
                                  ~btCollisionObject::CF_KINEMATIC_OBJECT);
    }
  } else {
    if (rb->kinematicMotionState) {
      rb->body->setMotionState(rb->kinematicMotionState.get());
      rb->body->setCollisionFlags(rb->body->getCollisionFlags() |
                                  btCollisionObject::CF_KINEMATIC_OBJECT);
      rb->body->setLinearVelocity(btVector3(0, 0, 0));
      rb->body->setAngularVelocity(btVector3(0, 0, 0));
    }
  }
}

void World::setAllDeactivated() {
  if (m_config.enableSleepDeactivation) {
    for (auto &rb : m_rigidBodies) {
      if (!rb.body)
        continue;
      if (rb.mode == 0)
        continue;
      rb.body->setActivationState(ISLAND_SLEEPING);
      rb.body->setMotionState(rb.kinematicMotionState.get());
      rb.body->setCollisionFlags(rb.body->getCollisionFlags() |
                                 btCollisionObject::CF_KINEMATIC_OBJECT);
      rb.body->setLinearVelocity(btVector3(0, 0, 0));
      rb.body->setAngularVelocity(btVector3(0, 0, 0));
    }
  }
}

void World::getActiveBoneIndices(std::vector<int> &outBoneIndices) const {
  outBoneIndices.clear();
  outBoneIndices.reserve(m_activeRigidBodyIndices.size());
  for (int idx : m_activeRigidBodyIndices) {
    if (idx < 0 || idx >= (int)m_rigidBodies.size())
      continue;
    const auto &rb = m_rigidBodies[idx];
    if (rb.boneIdx >= 0 && rb.body &&
        !(rb.body->getCollisionFlags() &
          btCollisionObject::CF_KINEMATIC_OBJECT)) {
      outBoneIndices.push_back(rb.boneIdx);
    }
  }
}
} // namespace mmbp::physics

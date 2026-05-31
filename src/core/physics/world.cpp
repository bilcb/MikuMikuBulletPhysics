#include "world.h"
#include "core/node/transform.h"
#include "core/math/converter.h"
#include <cstring>

namespace mmp::physics {

// ===== KinematicMotionState =====

KinematicMotionState::KinematicMotionState(const model::BoneNode* node, const btTransform& offset, float scale)
    : m_node(node), m_offset(offset), m_scale(scale) {}

void KinematicMotionState::getWorldTransform(btTransform& worldTransform) const {
    if (m_node) {
        btTransform boneGlobal = m_node->global;
        boneGlobal.setOrigin(mmp::math::applyScale(boneGlobal.getOrigin(), m_scale));
        worldTransform = boneGlobal * m_offset;
    } else {
        worldTransform = m_offset;
    }
}

void KinematicMotionState::setWorldTransform(const btTransform&) {
    // Kinematic: ignore physics writes
}

void KinematicMotionState::reset() {}

void KinematicMotionState::reflectGlobalTransform() {}

// ===== DynamicMotionState =====

DynamicMotionState::DynamicMotionState(const model::BoneNode* node, const btTransform& offset, float scale)
    : m_node(node), m_offset(offset), m_scale(scale) {
    m_invOffset = m_offset.inverse();
    reset();
}

void DynamicMotionState::getWorldTransform(btTransform& worldTransform) const {
    worldTransform = m_transform;
}

void DynamicMotionState::setWorldTransform(const btTransform& worldTransform) {
    m_transform = worldTransform;
}

void DynamicMotionState::reset() {
    if (m_node) {
        btTransform boneGlobal = m_node->global;
        boneGlobal.setOrigin(mmp::math::applyScale(boneGlobal.getOrigin(), m_scale));
        m_transform = boneGlobal * m_offset;
    }
}

void DynamicMotionState::reflectGlobalTransform() {
    if (!m_node) return;
    // Write back: global = physTransform * invOffset, then unscale
    const_cast<model::BoneNode*>(m_node)->global = m_transform * m_invOffset;
    const_cast<model::BoneNode*>(m_node)->global.setOrigin(
        mmp::math::removeScale(m_node->global.getOrigin(), m_scale));
}

// ===== DynamicBoneMergeMotionState =====

DynamicBoneMergeMotionState::DynamicBoneMergeMotionState(const model::BoneNode* node, const btTransform& offset, float scale)
    : m_node(node), m_offset(offset), m_scale(scale) {
    m_invOffset = m_offset.inverse();
    reset();
}

void DynamicBoneMergeMotionState::getWorldTransform(btTransform& worldTransform) const {
    worldTransform = m_transform;
}

void DynamicBoneMergeMotionState::setWorldTransform(const btTransform& worldTransform) {
    m_transform = worldTransform;
}

void DynamicBoneMergeMotionState::reset() {
    if (m_node) {
        btTransform boneGlobal = m_node->global;
        boneGlobal.setOrigin(mmp::math::applyScale(boneGlobal.getOrigin(), m_scale));
        m_transform = boneGlobal * m_offset;
    }
}

void DynamicBoneMergeMotionState::reflectGlobalTransform() {
    if (!m_node) return;
    // Like saba: preserve the bone's original position, only override rotation
    btTransform btGlobal = m_transform * m_invOffset;
    btGlobal.setOrigin(mmp::math::removeScale(btGlobal.getOrigin(), m_scale));
    // Preserve entire position from the bone's animation (not just Y)
    btVector3 origPos = m_node->global.getOrigin();
    btGlobal.setOrigin(origPos);
    const_cast<model::BoneNode*>(m_node)->global = btGlobal;
}

// ===== World =====

World::~World() {
    destroy();
}

bool World::create(const Config& cfg) {
    m_config = cfg;

    m_collisionCfg = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionCfg.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_dynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionCfg.get());

    m_dynamicsWorld->setGravity(m_config.gravity);
    m_dynamicsWorld->getSolverInfo().m_numIterations = m_config.solverIterations;

    // Ground plane (y=0, normal pointing up)
    m_groundShape = std::make_unique<btStaticPlaneShape>(btVector3(0, 1, 0), 0);
    m_groundMotionState = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundCI(0, m_groundMotionState.get(), m_groundShape.get());
    m_groundBody = std::make_unique<btRigidBody>(groundCI);
    m_dynamicsWorld->addRigidBody(m_groundBody.get());

    // Collision filter with ground proxy bypass
    m_filterCallback = std::make_unique<MMDFilterCallback>();
    m_filterCallback->m_nonFilterProxies.push_back(m_groundBody->getBroadphaseProxy());
    m_dynamicsWorld->getPairCache()->setOverlapFilterCallback(m_filterCallback.get());

    m_created = true;
    return true;
}

void World::destroy() {
    for (int i = (int)m_joints.size() - 1; i >= 0; i--) {
        if (m_dynamicsWorld) m_dynamicsWorld->removeConstraint(m_joints[i].get());
    }
    m_joints.clear();

    for (auto& rb : m_rigidBodies) {
        if (m_dynamicsWorld && rb.body) {
            m_dynamicsWorld->removeRigidBody(rb.body.get());
        }
    }
    m_rigidBodies.clear();

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

static std::unique_ptr<btCollisionShape> createShape(const pmx::PMXRigidBody& rbData, float scale) {
    const float s = scale;
    switch (rbData.shapeType) {
    case 0: {
        float radius = rbData.size.x() * s;
        return std::make_unique<btSphereShape>(radius);
    }
    case 1: {
        btVector3 half(rbData.size.x() * s, rbData.size.y() * s, rbData.size.z() * s);
        return std::make_unique<btBoxShape>(half);
    }
    case 2: {
        float radius = rbData.size.x() * s;
        float height = rbData.size.y() * s;
        return std::make_unique<btCapsuleShape>(radius, height);
    }
    default: {
        btVector3 half(rbData.size.x() * s, rbData.size.y() * s, rbData.size.z() * s);
        return std::make_unique<btBoxShape>(half);
    }
    }
}

int World::addRigidBody(const pmx::PMXRigidBody& rbData, int boneIdx, float scale,
                        const model::BoneNode* bone) {
    RigidBody rb;
    rb.boneIdx = boneIdx;
    rb.group = rbData.group;
    rb.groupMask = rbData.groupMask;
    rb.mode = rbData.mode;
    rb.shape = createShape(rbData, scale);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(mmp::math::applyScale(rbData.position, scale));
    startTransform.setRotation(rbData.rotation);

    // Compute offset matrix: bone_world_phys.inverse() * rb_world
    if (bone) {
        btTransform boneWorldPhys = bone->global;
        boneWorldPhys.setOrigin(mmp::math::applyScale(boneWorldPhys.getOrigin(), scale));
        rb.boneWorldInit = boneWorldPhys;
        rb.offsetMat = boneWorldPhys.inverse() * startTransform;
    }

    // Create motion states based on mode
    if (rbData.mode == 0) {
        // Static/Kinematic
        rb.kinematicMotionState = std::make_unique<KinematicMotionState>(bone, rb.offsetMat, scale);
    } else if (rbData.mode == 1) {
        // Dynamic
        rb.activeMotionState = std::make_unique<DynamicMotionState>(bone, rb.offsetMat, scale);
        rb.kinematicMotionState = std::make_unique<KinematicMotionState>(bone, rb.offsetMat, scale);
    } else if (rbData.mode == 2) {
        // Dynamic + Bone Merge
        rb.activeMotionState = std::make_unique<DynamicBoneMergeMotionState>(bone, rb.offsetMat, scale);
        rb.kinematicMotionState = std::make_unique<KinematicMotionState>(bone, rb.offsetMat, scale);
    }

    // Select initial motion state
    btMotionState* motionState = nullptr;
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

    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, rb.shape.get(), localInertia);
    ci.m_linearDamping = rbData.linearDamping;
    ci.m_angularDamping = rbData.angularDamping;
    ci.m_restitution = rbData.restitution;
    ci.m_friction = rbData.friction;
    ci.m_additionalDamping = true;

    rb.body = std::make_unique<btRigidBody>(ci);
    rb.body->setSleepingThresholds(0.01f, 0.001745f);
    rb.body->setActivationState(DISABLE_DEACTIVATION);
    rb.body->setUserPointer(&rb);

    if (isStatic) {
        rb.body->setCollisionFlags(rb.body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }

    m_dynamicsWorld->addRigidBody(rb.body.get(), 1 << rbData.group, rbData.groupMask);

    if (boneIdx >= (int)m_boneHasRb.size()) m_boneHasRb.resize(boneIdx + 1, 0);
    if (boneIdx >= 0) m_boneHasRb[boneIdx] = 1;

    int idx = (int)m_rigidBodies.size();
    m_rigidBodies.push_back(std::move(rb));
    return idx;
}

void World::addJoint(const pmx::PMXJoint& jointData, const RigidBody* rbA, const RigidBody* rbB) {
    if (!rbA || !rbB || !rbA->body || !rbB->body) return;

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

    {
        constexpr float kDefaultSpringDamping = 0.3f;
        float sp[3] = { jointData.springTranslate.x(), jointData.springTranslate.y(), jointData.springTranslate.z() };
        float sr[3] = { jointData.springRotate.x(), jointData.springRotate.y(), jointData.springRotate.z() };
        for (int i = 0; i < 3; i++) {
            if (sp[i] != 0.0f) {
                constraint->enableSpring(i, true);
                constraint->setStiffness(i, sp[i]);
                constraint->setDamping(i, kDefaultSpringDamping);
            }
            if (sr[i] != 0.0f) {
                constraint->enableSpring(i + 3, true);
                constraint->setStiffness(i + 3, sr[i]);
                constraint->setDamping(i + 3, kDefaultSpringDamping);
            }
        }
    }

    m_dynamicsWorld->addConstraint(constraint.get());
    m_joints.push_back(std::move(constraint));
}

void World::step(float delta) {
    if (m_dynamicsWorld) {
        m_dynamicsWorld->stepSimulation(delta, m_config.maxSubSteps, m_config.fixedTimestep);
    }
}

void World::resetPhysics(model::Model& model) {
    auto* cache = m_dynamicsWorld->getPairCache();
    auto* dispatcher = m_dynamicsWorld->getDispatcher();
    for (auto& rb : m_rigidBodies) {
        if (!rb.body || rb.boneIdx < 0) continue;
        auto* bone = model.getBone(rb.boneIdx);
        if (!bone) continue;
        // 清除 broadphase 碰撞对缓存（与 saba MMDRigidBody::Reset 一致）
        if (cache) {
            cache->cleanProxyFromPairs(rb.body->getBroadphaseHandle(), dispatcher);
        }
        rb.body->clearForces();
        rb.body->setLinearVelocity(btVector3(0,0,0));
        rb.body->setAngularVelocity(btVector3(0,0,0));
        // Reset motion state to current bone position
        if (rb.activeMotionState) rb.activeMotionState->reset();
        if (rb.kinematicMotionState) rb.kinematicMotionState->reset();
        // Apply kinematic transform
        if (rb.kinematicMotionState) {
            btTransform t;
            rb.kinematicMotionState->getWorldTransform(t);
            rb.body->setCenterOfMassTransform(t);
        }
    }
}

void World::syncBoneTransforms(model::Model& model) {
    for (auto& rb : m_rigidBodies) {
        if (!rb.body || rb.boneIdx < 0) continue;
        auto* bone = model.getBone(rb.boneIdx);
        if (!bone) continue;

        // Use the rigid body's ACTUAL world transform (not the motion state's interpolated value)
        if (rb.activeMotionState) {
            btTransform physTransform = rb.body->getWorldTransform();
            rb.activeMotionState->setWorldTransform(physTransform);
            rb.activeMotionState->reflectGlobalTransform();
        }

        // Cascade to child transforms (skip children that have their own rb)
        cascadeChildTransforms(model, rb.boneIdx);
    }
}

void World::cascadeChildTransforms(model::Model& model, int boneIdx) {
    auto* bone = model.getBone(boneIdx);
    if (!bone) return;
    for (uint32_t childIdx : bone->children) {
        auto* child = model.getBone(childIdx);
        if (!child) continue;
        // Skip children that have their own rigid body (they are synced directly)
        if (childIdx < m_boneHasRb.size() && m_boneHasRb[childIdx]) {
            // Still cascade through this child's children
            cascadeChildTransforms(model, childIdx);
            continue;
        }
        node::updateLocalTransform(*child);
        child->global = bone->global * child->local;
        cascadeChildTransforms(model, childIdx);
    }
}

void World::setActivation(int idx, bool active) {
    auto* rb = getRigidBody(idx);
    if (!rb || !rb->body) return;

    if (rb->mode == 0) {
        // Static: always kinematic
        rb->body->setMotionState(rb->kinematicMotionState.get());
        return;
    }

    if (active) {
        // Switch to dynamic
        if (rb->activeMotionState) {
            // Capture current world position before switching motion state.
            // If body is kinematic, refresh from the animated bone position first.
            btTransform savedWorld;
            if (rb->body->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) {
                rb->kinematicMotionState->getWorldTransform(savedWorld);
            } else {
                savedWorld = rb->body->getWorldTransform();
            }

            rb->activeMotionState->reset();
            rb->body->setMotionState(rb->activeMotionState.get());
            // Restore world transform: setMotionState reads from motion state and overwrites,
            // but we need Bullet's internal physics transform (which includes accumulated velocity)
            rb->body->setWorldTransform(savedWorld);
            rb->body->setCollisionFlags(rb->body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
        }
    } else {
        // Switch to kinematic
        if (rb->kinematicMotionState) {
            rb->body->setMotionState(rb->kinematicMotionState.get());
            rb->body->setCollisionFlags(rb->body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        }
    }
}

} // namespace mmp::physics

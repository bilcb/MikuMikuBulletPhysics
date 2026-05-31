#include "transform.h"

namespace mmp::node {

void updateLocalTransform(model::BoneNode& bone) {
    btTransform t;
    t.setIdentity();
    btVector3 origin = bone.animTranslate + bone.ikTranslate + bone.restPosition;
    btQuaternion rot = bone.ikRotate * bone.animRotate;  // saba: ikRotate before animRotate
    // Apply append transforms
    if (bone.hasAppendTranslate) origin += bone.appendTranslate;
    if (bone.hasAppendRotate) rot = rot * bone.appendRotate;
    t.setOrigin(origin);
    t.setRotation(rot);
    bone.local = t;
}

void updateGlobalTransform(model::BoneNode& bone, model::Model& model) {
    if (bone.parentIdx >= 0) {
        auto* parent = model.getBone(bone.parentIdx);
        if (parent) bone.global = parent->global * bone.local;
        else bone.global = bone.local;
    } else {
        bone.global = bone.local;
    }
    // Recursively update children (matching saba)
    for (uint32_t childIdx : bone.children) {
        auto* child = model.getBone(childIdx);
        if (child) {
            updateLocalTransform(*child);
            updateGlobalTransform(*child, model);
        }
    }
}

// Non-recursive: updates global for ONLY bones matching the filter, without cascading through all children
void updateFilteredGlobalTransform(model::BoneNode& bone, model::Model& model, bool afterPhysics) {
    if (bone.parentIdx >= 0) {
        auto* parent = model.getBone(bone.parentIdx);
        if (parent) bone.global = parent->global * bone.local;
        else bone.global = bone.local;
    } else {
        bone.global = bone.local;
    }
    for (uint32_t childIdx : bone.children) {
        auto* child = model.getBone(childIdx);
        if (child) {
            if (child->transAfterPhys == afterPhysics) {
                updateLocalTransform(*child);
                updateFilteredGlobalTransform(*child, model, afterPhysics);
            }
        }
    }
}

void updateChildTransforms(model::BoneNode& bone, model::Model& model) {
    for (uint32_t childIdx : bone.children) {
        auto* child = model.getBone(childIdx);
        if (child) {
            updateLocalTransform(*child);
            child->global = bone.global * child->local;
            updateChildTransforms(*child, model);
        }
    }
}

void updateAppendTransforms(model::Model& model) {
    for (size_t i = 0; i < model.boneCount(); i++) {
        auto* bone = model.getBone(i);
        if (!bone || bone->appendBoneIdx < 0) continue;
        auto* src = model.getBone(bone->appendBoneIdx);
        if (!src) continue;
        // Append translate: delta from rest position (like saba: GetTranslate() - GetInitialTranslate())
        btVector3 srcTrans = src->animTranslate;
        // Append rotate: source's full animated rotation including IK
        btQuaternion srcRot = src->ikRotate * src->animRotate;
        // Weighted application
        if (bone->hasAppendTranslate)
            bone->appendTranslate = srcTrans * bone->appendWeight;
        if (bone->hasAppendRotate)
            bone->appendRotate = btQuaternion::getIdentity().slerp(srcRot, bone->appendWeight);
    }
}

} // namespace mmp::node

#include "evaluator.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace mmp::anim {

struct VMDBezier {
    float cp1x, cp1y, cp2x, cp2y;
    VMDBezier() : cp1x(0), cp1y(0), cp2x(1), cp2y(1) {}
    void set(const uint8_t* cp) {
        cp1x = cp[0] / 127.0f;  cp1y = cp[4] / 127.0f;
        cp2x = cp[8] / 127.0f;  cp2y = cp[12] / 127.0f;
    }
    float evalX(float t) const {
        float t2 = t * t, t3 = t2 * t, u = 1.0f - t, u2 = u * u;
        return t3 + 3.0f * t2 * u * cp2x + 3.0f * t * u2 * cp1x;
    }
    float evalY(float t) const {
        float t2 = t * t, t3 = t2 * t, u = 1.0f - t, u2 = u * u;
        return t3 + 3.0f * t2 * u * cp2y + 3.0f * t * u2 * cp1y;
    }
    float findX(float time) const {
        const float eps = 0.00001f;
        float lo = 0.0f, hi = 1.0f, t = 0.5f, x = evalX(t);
        while (std::abs(time - x) > eps) {
            if (time < x) hi = t; else lo = t;
            t = (hi + lo) * 0.5f; x = evalX(t);
        }
        return t;
    }
};

// Find the keyframe at or just after the given time, with cached start index
// Matches saba's FindBoundKey template
template<typename KeyType>
static size_t findBoundKey(const std::vector<KeyType*>& keys, uint32_t frame, size_t startIdx) {
    if (keys.empty() || startIdx >= keys.size()) return keys.size();

    // Check if cached index is still valid
    if (startIdx < keys.size() && keys[startIdx]->frame <= frame) {
        if (startIdx + 1 < keys.size() && keys[startIdx + 1]->frame > frame) {
            return startIdx + 1;
        }
    } else if (startIdx > 0 && keys[startIdx - 1]->frame <= frame) {
        if (startIdx < keys.size() && keys[startIdx]->frame > frame) {
            return startIdx;
        }
    }

    // Binary search
    auto it = std::upper_bound(keys.begin(), keys.end(), frame,
        [](uint32_t f, const vmd::VMDKeyframe* kf) { return f < kf->frame; });
    return std::distance(keys.begin(), it);
}

template<typename KeyType>
static size_t findBoundKeyMorph(const std::vector<KeyType*>& keys, uint32_t frame, size_t startIdx) {
    if (keys.empty() || startIdx >= keys.size()) return keys.size();

    if (startIdx < keys.size() && keys[startIdx]->frame <= frame) {
        if (startIdx + 1 < keys.size() && keys[startIdx + 1]->frame > frame) {
            return startIdx + 1;
        }
    } else if (startIdx > 0 && keys[startIdx - 1]->frame <= frame) {
        if (startIdx < keys.size() && keys[startIdx]->frame > frame) {
            return startIdx;
        }
    }

    auto it = std::upper_bound(keys.begin(), keys.end(), frame,
        [](uint32_t f, const vmd::MorphKeyframe* kf) { return f < kf->frame; });
    return std::distance(keys.begin(), it);
}

// Per-bone controller with cached key index
struct BoneController {
    std::vector<const vmd::VMDKeyframe*> keys;
    size_t startKeyIndex = 0;

    void evaluate(model::BoneNode* bone, float frame) {
        if (keys.empty()) return;

        size_t idx = findBoundKey(keys, (uint32_t)frame, startKeyIndex);
        if (idx > 0 && idx <= keys.size()) {
            startKeyIndex = idx - 1;
        }

        const vmd::VMDKeyframe* k0 = nullptr;
        const vmd::VMDKeyframe* k1 = nullptr;

        if (idx == 0) {
            k1 = keys[0];
        } else if (idx >= keys.size()) {
            k0 = keys[keys.size() - 1];
        } else {
            k0 = keys[idx - 1];
            k1 = keys[idx];
        }

        if (!k0 && !k1) return;
        if (!k1 || k0 == k1) {
            bone->animTranslate = k0->translation;
            bone->animRotate = k0->rotation;
            return;
        }
        if (!k0) {
            bone->animTranslate = k1->translation;
            bone->animRotate = k1->rotation;
            return;
        }

        float t = (frame - k0->frame) / (float)(k1->frame - k0->frame);
        t = std::max(0.0f, std::min(1.0f, t));

        VMDBezier bx, by, bz, br;
        bx.set(k0->interpX); by.set(k0->interpY); bz.set(k0->interpZ); br.set(k0->interpRot);
        float tx = bx.evalY(bx.findX(t));
        float ty = by.evalY(by.findX(t));
        float tz = bz.evalY(bz.findX(t));
        float tr = br.evalY(br.findX(t));

        bone->animTranslate = btVector3(
            k0->translation.x() + (k1->translation.x() - k0->translation.x()) * tx,
            k0->translation.y() + (k1->translation.y() - k0->translation.y()) * ty,
            k0->translation.z() + (k1->translation.z() - k0->translation.z()) * tz);
        bone->animRotate = k0->rotation.slerp(k1->rotation, tr);
    }
};

// Per-morph controller with cached key index
struct MorphController {
    std::vector<const vmd::MorphKeyframe*> keys;
    size_t startKeyIndex = 0;

    void evaluate(float* weightPtr, float frame) {
        if (keys.empty() || !weightPtr) return;

        size_t idx = findBoundKeyMorph(keys, (uint32_t)frame, startKeyIndex);
        if (idx > 0 && idx <= keys.size()) {
            startKeyIndex = idx - 1;
        }

        const vmd::MorphKeyframe* k0 = nullptr;
        const vmd::MorphKeyframe* k1 = nullptr;

        if (idx == 0) {
            k1 = keys[0];
        } else if (idx >= keys.size()) {
            k0 = keys[keys.size() - 1];
        } else {
            k0 = keys[idx - 1];
            k1 = keys[idx];
        }

        if (!k0 && !k1) return;
        if (!k1) { *weightPtr = k0->weight; return; }
        if (!k0) { *weightPtr = k1->weight; return; }

        float t = (frame - k0->frame) / (float)(k1->frame - k0->frame);
        t = std::max(0.0f, std::min(1.0f, t));
        *weightPtr = k0->weight + (k1->weight - k0->weight) * t;
    }
};

void evaluate(const vmd::VMDData& vmd, model::Model& model, float frame) {
    // Build per-bone controllers (sorted by frame)
    static thread_local std::unordered_map<std::string, BoneController> boneControllers;
    static thread_local const vmd::VMDKeyframe* lastKeyframeData = nullptr;
    static thread_local size_t lastVmdSize = 0;

    // Rebuild if VMD data changed (check both data pointer and size)
    const vmd::VMDKeyframe* curData = vmd.keyframes.empty() ? nullptr : vmd.keyframes.data();
    if (curData != lastKeyframeData || vmd.keyframes.size() != lastVmdSize) {
        boneControllers.clear();
        for (const auto& kf : vmd.keyframes) {
            boneControllers[kf.boneName].keys.push_back(&kf);
        }
        for (auto& [name, ctrl] : boneControllers) {
            std::sort(ctrl.keys.begin(), ctrl.keys.end(),
                [](const vmd::VMDKeyframe* a, const vmd::VMDKeyframe* b) { return a->frame < b->frame; });
            ctrl.startKeyIndex = 0;
        }
        lastKeyframeData = curData;
        lastVmdSize = vmd.keyframes.size();
    }

    for (auto& [name, ctrl] : boneControllers) {
        auto* bone = model.getBone(name);
        if (!bone) continue;
        ctrl.evaluate(bone, frame);
    }
}

void evaluateMorphs(const vmd::VMDData& vmd, model::Model& model, float frame) {
    // Build per-morph controllers (sorted by frame)
    static thread_local std::unordered_map<std::string, MorphController> morphControllers;
    static thread_local const vmd::MorphKeyframe* lastMorphData = nullptr;
    static thread_local size_t lastMorphSize = 0;

    const vmd::MorphKeyframe* curData = vmd.morphKeys.empty() ? nullptr : vmd.morphKeys.data();
    if (curData != lastMorphData || vmd.morphKeys.size() != lastMorphSize) {
        morphControllers.clear();
        for (const auto& mk : vmd.morphKeys) {
            morphControllers[mk.name].keys.push_back(&mk);
        }
        for (auto& [name, ctrl] : morphControllers) {
            std::sort(ctrl.keys.begin(), ctrl.keys.end(),
                [](const vmd::MorphKeyframe* a, const vmd::MorphKeyframe* b) { return a->frame < b->frame; });
            ctrl.startKeyIndex = 0;
        }
        lastMorphData = curData;
        lastMorphSize = vmd.morphKeys.size();
    }

    for (auto& [name, ctrl] : morphControllers) {
        int idx = model.findMorphIndex(name);
        if (idx < 0) continue;
        float* wp = model.getMorphWeightPtr(idx);
        ctrl.evaluate(wp, frame);
    }
}

void evaluateIKKeys(const vmd::VMDData& vmd, model::Model& model, float frame) {
    for (const auto& ik : vmd.ikKeys) {
        if (ik.frame > (uint32_t)frame) break;
        for (const auto& st : ik.states) {
            auto* bone = model.getBone(st.name);
            if (bone && bone->isIK) bone->ikEnabled = st.enabled;
        }
    }
}

} // namespace mmp::anim

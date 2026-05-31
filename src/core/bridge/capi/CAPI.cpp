// C API implementation — exception-safe with parameter validation
#include "CAPI.h"
#include "../engine/Engine.h"
#include "../adapter/BlenderAdapter.h"
#include "core/physics/world.h"
#include <cstdio>

static mmp::bridge::Engine* cast(void* ctx) {
    return reinterpret_cast<mmp::bridge::Engine*>(ctx);
}

MMP_API void* mmp_create() {
    try {
        return new mmp::bridge::Engine();
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_create failed: %s\n", e.what());
        return nullptr;
    }
}

MMP_API void mmp_destroy(void* ctx) {
    if (!ctx) return;
    try {
        delete cast(ctx);
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_destroy exception: %s\n", e.what());
    }
}

MMP_API int mmp_load_pmx(void* ctx, const char* path) {
    if (!ctx || !path) return 0;
    try {
        return cast(ctx)->loadPMXFile(path) ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_load_pmx exception: %s\n", e.what());
        return 0;
    }
}

MMP_API int mmp_load_pmx_mem(void* ctx, const uint8_t* data, int size) {
    if (!ctx || !data || size <= 0) return 0;
    try {
        return cast(ctx)->loadPMX(data, size) ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_load_pmx_mem exception: %s\n", e.what());
        return 0;
    }
}

MMP_API int mmp_load_vmd(void* ctx, const char* path) {
    if (!ctx || !path) return 0;
    try {
        return cast(ctx)->loadVMDFile(path) ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_load_vmd exception: %s\n", e.what());
        return 0;
    }
}

MMP_API int mmp_load_vmd_mem(void* ctx, const uint8_t* data, int size) {
    if (!ctx || !data || size <= 0) return 0;
    try {
        return cast(ctx)->loadVMD(data, size) ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_load_vmd_mem exception: %s\n", e.what());
        return 0;
    }
}

MMP_API int mmp_build_physics(void* ctx) {
    if (!ctx) return 0;
    try {
        return cast(ctx)->buildPhysics() ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_build_physics exception: %s\n", e.what());
        return 0;
    }
}

MMP_API void mmp_set_scale(void* ctx, float scale) {
    if (!ctx) return;
    try {
        auto* engine = cast(ctx);
        auto cfg = engine->getPhysicsConfig();
        cfg.scale = scale;
        engine->setPhysicsConfig(cfg);
    } catch (...) {}
}

MMP_API void mmp_set_gravity(void* ctx, float x, float y, float z) {
    if (!ctx) return;
    try {
        auto* engine = cast(ctx);
        auto cfg = engine->getPhysicsConfig();
        cfg.gravity = btVector3(x, y, z);
        engine->setPhysicsConfig(cfg);
    } catch (...) {}
}

MMP_API void mmp_set_solver_iters(void* ctx, int n) {
    if (!ctx) return;
    try {
        auto* engine = cast(ctx);
        auto cfg = engine->getPhysicsConfig();
        cfg.solverIterations = n;
        engine->setPhysicsConfig(cfg);
    } catch (...) {}
}

MMP_API void mmp_set_fixed_timestep(void* ctx, float fps) {
    if (!ctx) return;
    try {
        auto* engine = cast(ctx);
        auto cfg = engine->getPhysicsConfig();
        cfg.fixedTimestep = fps;
        engine->setPhysicsConfig(cfg);
    } catch (...) {}
}

MMP_API void mmp_set_max_substeps(void* ctx, int n) {
    if (!ctx) return;
    try {
        auto* engine = cast(ctx);
        auto cfg = engine->getPhysicsConfig();
        cfg.maxSubSteps = n;
        engine->setPhysicsConfig(cfg);
    } catch (...) {}
}

MMP_API void mmp_set_bone_rest_info(void* ctx, const float* mat3, int count) {
    if (!ctx || !mat3 || count <= 0) return;
    try { cast(ctx)->setBoneRestInfo(mat3, count); } catch (...) {}
}

MMP_API void mmp_set_bone_rest_positions(void* ctx, const float* offset, int count) {
    if (!ctx || !offset || count <= 0) return;
    try { cast(ctx)->setBoneRestPositions(offset, count); } catch (...) {}
}

MMP_API void mmp_compute_rest_info(const float* bone_local_16, float* out_mat3x3_9, float* out_offset_3) {
    if (!bone_local_16 || !out_mat3x3_9 || !out_offset_3) return;
    try {
        btMatrix3x3 boneLocal(
            bone_local_16[0], bone_local_16[1], bone_local_16[2],
            bone_local_16[4], bone_local_16[5], bone_local_16[6],
            bone_local_16[8], bone_local_16[9], bone_local_16[10]);
        btVector3 restPos(bone_local_16[3], bone_local_16[7], bone_local_16[11]);
        btMatrix3x3 restMat = mmp::bridge::BlenderAdapter::computeBoneRestMatrix(boneLocal);
        btVector3 restOffset = mmp::bridge::BlenderAdapter::computeBoneRestOffset(boneLocal, restPos);
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++)
            out_mat3x3_9[r * 3 + c] = restMat[r][c];
        out_offset_3[0] = restOffset.x(); out_offset_3[1] = restOffset.y(); out_offset_3[2] = restOffset.z();
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_compute_rest_info exception: %s\n", e.what());
    }
}

MMP_API int mmp_step(void* ctx, float frame, float delta, float* out_locs, float* out_quats, int bone_count) {
    if (!ctx || !out_locs || !out_quats || bone_count <= 0) return 0;
    try {
        return cast(ctx)->step(frame, delta, out_locs, out_quats, bone_count);
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_step exception: %s\n", e.what());
        return 0;
    }
}

MMP_API void mmp_eval_animation(void* ctx, float frame) {
    if (!ctx) return;
    try { cast(ctx)->evalAnimation(frame); } catch (...) {}
}

MMP_API void mmp_solve_ik(void* ctx) {
    if (!ctx) return;
    try { cast(ctx)->solveIK(); } catch (...) {}
}

MMP_API void mmp_step_physics(void* ctx, float frame, float delta) {
    if (!ctx) return;
    try { cast(ctx)->stepPhysics(frame, delta); } catch (...) {}
}

MMP_API void mmp_get_bone_world_mat4(void* ctx, int idx, float* out_mat4) {
    if (!ctx || !out_mat4) return;
    try { cast(ctx)->getBoneWorldMat4(idx, out_mat4); } catch (...) {}
}

MMP_API int mmp_get_ik_count(void* ctx) {
    if (!ctx) return 0;
    try { return cast(ctx)->getIKCount(); } catch (...) { return 0; }
}

MMP_API void mmp_get_solver_info(void* ctx, float* out) {
    if (!ctx || !out) return;
    try { cast(ctx)->getSolverInfo(out); } catch (...) {}
}

MMP_API int mmp_get_bone_count(void* ctx) {
    if (!ctx) return 0;
    try { return cast(ctx)->getBoneCount(); } catch (...) { return 0; }
}

MMP_API int mmp_get_bone_name(void* ctx, int idx, char* out, int max_len) {
    if (!ctx || !out || max_len <= 0) return 0;
    try {
        return cast(ctx)->getBoneName(idx, out, max_len);
    } catch (const std::exception& e) {
        fprintf(stderr, "[MMP] mmp_get_bone_name exception: %s\n", e.what());
        return 0;
    }
}

MMP_API int mmp_get_rigid_body_count(void* ctx) {
    if (!ctx) return 0;
    try { return cast(ctx)->getRigidBodyCount(); } catch (...) { return 0; }
}

MMP_API int mmp_get_joint_count(void* ctx) {
    if (!ctx) return 0;
    try { return cast(ctx)->getJointCount(); } catch (...) { return 0; }
}

MMP_API const char* mmp_get_last_error(void* ctx) {
    if (!ctx) return "";
    try { return cast(ctx)->getLastError(); } catch (...) { return ""; }
}

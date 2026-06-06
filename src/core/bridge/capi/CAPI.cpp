
#include "CAPI.h"
#include "../adapter/BlenderAdapter.h"
#include "../engine/Engine.h"
#include "core/physics/world.h"
#include "core/util/logger.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define CAPI_TRY_INT(ctx, expr)                                                \
  do {                                                                         \
    if (!(ctx))                                                                \
      return 0;                                                                \
    auto *_e = cast(ctx);                                                      \
    if (!_e)                                                                   \
      return 0;                                                                \
    try {                                                                      \
      return (expr);                                                           \
    } catch (const std::exception &ex) {                                       \
      _e->setLastError(ex.what());                                             \
      return 0;                                                                \
    }                                                                          \
  } while (0)

#define CAPI_TRY_INT_CODE(ctx, code, expr)                                     \
  do {                                                                         \
    if (!(ctx))                                                                \
      return 0;                                                                \
    auto *_e = cast(ctx);                                                      \
    if (!_e)                                                                   \
      return 0;                                                                \
    try {                                                                      \
      return (expr);                                                           \
    } catch (const std::exception &ex) {                                       \
      _e->setLastError((code), ex.what());                                     \
      return 0;                                                                \
    }                                                                          \
  } while (0)

#define CAPI_TRY_VOID(ctx, expr)                                               \
  do {                                                                         \
    if (!(ctx))                                                                \
      return;                                                                  \
    auto *_e = cast(ctx);                                                      \
    if (!_e)                                                                   \
      return;                                                                  \
    try {                                                                      \
      (expr);                                                                  \
    } catch (...) {                                                            \
    }                                                                          \
  } while (0)

static constexpr uint32_t kEngineMagic = 0x4D4D5048;

struct EngineHandle {
  uint32_t magic;
  mmbp::bridge::Engine *engine;
};

static mmbp::bridge::Engine *cast(void *ctx) {
  if (!ctx)
    return nullptr;
  auto *handle = static_cast<EngineHandle *>(ctx);
  if (handle->magic != kEngineMagic) {
    g_logger.warn("[MMBP] invalid engine handle (magic mismatch)");
    return nullptr;
  }
  return handle->engine;
}

MMBP_API void *mmbp_create() {
  try {
    auto *engine = new mmbp::bridge::Engine();
    auto *handle = new EngineHandle{kEngineMagic, engine};
    return handle;
  } catch (const std::exception &e) {
    g_logger.error("[MMBP] mmbp_create failed: %s", e.what());
    return nullptr;
  }
}

MMBP_API void mmbp_destroy(void *ctx) {
  if (!ctx)
    return;
  try {
    auto *handle = static_cast<EngineHandle *>(ctx);
    if (handle->magic != kEngineMagic)
      return;
    handle->magic = 0;
    delete handle->engine;
    delete handle;
  } catch (const std::exception &e) {
    g_logger.error("[MMBP] mmbp_destroy exception: %s", e.what());
  }
}

MMBP_API int mmbp_load_pmx(void *ctx, const char *path) {
  if (!ctx || !path)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->loadPMXFile(path) ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_load_pmx_mem(void *ctx, const uint8_t *data, int size) {
  if (!ctx || !data || size <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->loadPMX(data, size) ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_load_vmd(void *ctx, const char *path) {
  if (!ctx || !path)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->loadVMDFile(path) ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_load_vpd(void *ctx, const char *path) {
  if (!ctx || !path)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->loadVPDFile(path) ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_load_vmd_mem(void *ctx, const uint8_t *data, int size) {
  if (!ctx || !data || size <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->loadVMD(data, size) ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_merge_vmd_mem(void *ctx, const uint8_t *data, int size) {
  if (!ctx || !data || size <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->mergeVMD(data, size) ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_build_physics(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->buildPhysics() ? 1 : 0;
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API void mmbp_set_scale(void *ctx, float scale) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigScale(scale);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_gravity(void *ctx, float x, float y, float z) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigGravity(x, y, z);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_solver_iters(void *ctx, int n) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigSolverIters(n);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_fixed_timestep(void *ctx, float fps) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigFixedTimestep(fps);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_max_substeps(void *ctx, int n) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigMaxSubsteps(n);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_spring_damping(void *ctx, float damping) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigSpringDamping(damping);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_spring_damping_linear(void *ctx, int axis,
                                             float damping) {
  if (!ctx || axis < 0 || axis > 2)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigSpringDampingLinear(axis, damping);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_spring_damping_angular(void *ctx, int axis,
                                              float damping) {
  if (!ctx || axis < 0 || axis > 2)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigSpringDampingAngular(axis, damping);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_bone_rest_info(void *ctx, const float *mat3, int count) {
  if (!ctx || !mat3 || count <= 0)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setBoneRestInfo(mat3, count);
  } catch (...) {
  }
}

MMBP_API void mmbp_compute_rest_info(const float *bone_local_16,
                                     float *out_mat3x3_9, float *out_offset_3) {
  if (!bone_local_16 || !out_mat3x3_9)
    return;

  bool finite = true;
  for (int i = 0; i < 12; i++) {
    if (!std::isfinite(bone_local_16[i])) {
      finite = false;
      break;
    }
  }
  if (!finite) {
    g_logger.error("[MMBP] mmbp_compute_rest_info: non-finite input (NaN/Inf), "
                   "zeroing output");
    btMatrix3x3 zero(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        out_mat3x3_9[r * 3 + c] = zero[r][c];
    if (out_offset_3)
      out_offset_3[0] = out_offset_3[1] = out_offset_3[2] = 0.0f;
    return;
  }
  try {
    btMatrix3x3 boneLocal(bone_local_16[0], bone_local_16[1], bone_local_16[2],
                          bone_local_16[4], bone_local_16[5], bone_local_16[6],
                          bone_local_16[8], bone_local_16[9],
                          bone_local_16[10]);
    btMatrix3x3 restMat =
        mmbp::bridge::BlenderAdapter::computeBoneRestMatrix(boneLocal);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        out_mat3x3_9[r * 3 + c] = restMat[r][c];

    if (out_offset_3) {
      out_offset_3[0] = bone_local_16[3];
      out_offset_3[1] = bone_local_16[7];
      out_offset_3[2] = bone_local_16[11];
    }
  } catch (const std::exception &e) {
    g_logger.error("[MMBP] mmbp_compute_rest_info exception: %s", e.what());
  }
}

MMBP_API int mmbp_step(void *ctx, float frame, float delta, float *out_locs,
                       float *out_quats, int bone_count) {
  if (!ctx || !out_locs || !out_quats || bone_count <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {

    return engine->step(frame, delta, out_locs, out_quats, bone_count);
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_reset_to_animation(void *ctx, float frame, float *out_locs,
                                     float *out_quats, int bone_count) {
  if (!ctx || !out_locs || !out_quats || bone_count <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->resetToAnimation(frame, out_locs, out_quats, bone_count);
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_eval_only(void *ctx, float frame, float *out_locs,
                            float *out_quats, int bone_count) {
  if (!ctx || !out_locs || !out_quats || bone_count <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->evalOnly(frame, out_locs, out_quats, bone_count);
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API void mmbp_eval_animation(void *ctx, float frame) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->evalAnimation(frame);
  } catch (...) {
  }
}

MMBP_API void mmbp_solve_ik(void *ctx) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->solveIK();
  } catch (...) {
  }
}

MMBP_API void mmbp_step_physics(void *ctx, float frame, float delta) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->stepPhysics(frame, delta);
  } catch (...) {
  }
}

MMBP_API void mmbp_sync_physics(void *ctx, float frame, float delta,
                                int frame_count) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->syncPhysics(frame, delta, frame_count);
  } catch (...) {
  }
}

MMBP_API void mmbp_get_bone_world_mat4(void *ctx, int idx, float *out_mat4) {
  if (!ctx || !out_mat4)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->getBoneWorldMat4(idx, out_mat4);
  } catch (...) {
  }
}

MMBP_API int mmbp_get_ik_count(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->getIKCount();
  } catch (...) {
    return 0;
  }
}

MMBP_API void mmbp_get_solver_info(void *ctx, float *out) {
  if (!ctx || !out)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->getSolverInfo(out);
  } catch (...) {
  }
}

MMBP_API int mmbp_get_bone_count(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->getBoneCount();
  } catch (...) {
    return 0;
  }
}

MMBP_API int mmbp_get_bone_name(void *ctx, int idx, char *out, int max_len) {
  if (!ctx || !out || max_len <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->getBoneName(idx, out, max_len);
  } catch (const std::exception &e) {
    engine->setLastError(e.what());
    return 0;
  }
}

MMBP_API int mmbp_get_all_bone_names(void *ctx, char *names_buf,
                                     int name_buf_size) {
  if (!ctx || !names_buf || name_buf_size <= 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    int count = engine->getBoneCount();
    for (int i = 0; i < count; i++) {
      engine->getBoneName(i, names_buf + i * name_buf_size, name_buf_size);
    }
    return count;
  } catch (...) {
    return 0;
  }
}

MMBP_API int mmbp_get_rigid_body_count(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->getRigidBodyCount();
  } catch (...) {
    return 0;
  }
}

MMBP_API int mmbp_get_joint_count(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->getJointCount();
  } catch (...) {
    return 0;
  }
}

MMBP_API float mmbp_get_fixed_timestep(void *ctx) {
  if (!ctx)
    return 1.0f / 120.0f;
  auto *engine = cast(ctx);
  if (!engine)
    return 1.0f / 120.0f;
  try {
    return engine->getPhysicsConfig().fixedTimestep;
  } catch (...) {
    return 1.0f / 120.0f;
  }
}

MMBP_API const char *mmbp_get_last_error(void *ctx) {
  if (!ctx)
    return "";
  auto *engine = cast(ctx);
  if (!engine)
    return "";
  try {
    return engine->getLastError();
  } catch (...) {
    return "";
  }
}

MMBP_API int mmbp_get_last_error_code(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return static_cast<int>(engine->getLastErrorCode());
  } catch (...) {
    return 0;
  }
}

MMBP_API int mmbp_get_warning_count(void *ctx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return static_cast<int>(engine->getWarnings().size());
  } catch (...) {
    return 0;
  }
}

MMBP_API int mmbp_get_warning(void *ctx, int idx, char *out, int max_len) {
  if (!ctx || !out || max_len <= 0 || idx < 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    const auto &warnings = engine->getWarnings();
    if (idx >= static_cast<int>(warnings.size()))
      return 0;
    const auto &w = warnings[idx];
    int copyLen = std::min(static_cast<int>(w.size()), max_len - 1);
    std::memcpy(out, w.data(), copyLen);
    out[copyLen] = '\0';
    return 1;
  } catch (...) {
    return 0;
  }
}

MMBP_API void mmbp_set_log_callback(void (*callback)(int level,
                                                     const char *message,
                                                     void *userData),
                                    void *userData) {
  mmbp::log::setLogCallback(callback, userData);
}

MMBP_API void mmbp_set_log_level(int level) {
  mmbp::log::setLogLevel(static_cast<mmbp::log::Level>(level));
}

MMBP_API void mmbp_set_ik_loop_count(void *ctx, int bone_idx, int loop_count) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setIKLoopCount(bone_idx, loop_count);
  } catch (...) {
  }
}

MMBP_API void mmbp_set_ik_max_step_angle(void *ctx, float angle) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setIKMaxStepAngle(angle);
  } catch (...) {
  }
}

MMBP_API int mmbp_get_ik_loop_count(void *ctx, int bone_idx) {
  if (!ctx)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    return engine->getIKLoopCount(bone_idx);
  } catch (...) {
    return 0;
  }
}

MMBP_API void mmbp_set_sleep_deactivation(void *ctx, int enable) {
  if (!ctx)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    engine->setConfigSleepDeactivation(enable != 0);
  } catch (...) {
  }
}

MMBP_API int mmbp_bone_has_rigid_body(void *ctx, int bone_idx) {
  if (!ctx || bone_idx < 0)
    return 0;
  auto *engine = cast(ctx);
  if (!engine)
    return 0;
  try {
    auto *world = engine->getPhysicsWorld();
    return world && world->boneHasRigidBody(bone_idx) ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

MMBP_API void mmbp_get_active_bone_indices(void *ctx, int *out_indices,
                                           int max_count) {
  if (!ctx || !out_indices || max_count <= 0)
    return;
  auto *engine = cast(ctx);
  if (!engine)
    return;
  try {
    auto *world = engine->getPhysicsWorld();
    if (!world)
      return;
    std::vector<int> indices;
    world->getActiveBoneIndices(indices);
    int copy = std::min((int)indices.size(), max_count);
    for (int i = 0; i < copy; i++)
      out_indices[i] = indices[i];
  } catch (...) {
  }
} // namespace mmbp::bridge

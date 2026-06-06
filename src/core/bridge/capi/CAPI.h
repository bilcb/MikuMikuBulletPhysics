#pragma once
#include <cstdint>

#ifdef _WIN32
#define MMBP_API extern "C" __declspec(dllexport)
#else
#define MMBP_API extern "C"
#endif

MMBP_API void *mmbp_create();
MMBP_API void mmbp_destroy(void *ctx);
MMBP_API int mmbp_load_pmx(void *ctx, const char *path);
MMBP_API int mmbp_load_pmx_mem(void *ctx, const uint8_t *data, int size);
MMBP_API int mmbp_load_vmd(void *ctx, const char *path);
MMBP_API int mmbp_load_vpd(void *ctx, const char *path);
MMBP_API int mmbp_load_vmd_mem(void *ctx, const uint8_t *data, int size);
MMBP_API int mmbp_merge_vmd_mem(void *ctx, const uint8_t *data, int size);
MMBP_API int mmbp_build_physics(void *ctx);
MMBP_API void mmbp_set_scale(void *ctx, float scale);
MMBP_API void mmbp_set_gravity(void *ctx, float x, float y, float z);
MMBP_API void mmbp_set_solver_iters(void *ctx, int n);
MMBP_API void mmbp_set_fixed_timestep(void *ctx, float fps);
MMBP_API void mmbp_set_max_substeps(void *ctx, int n);
MMBP_API void mmbp_set_spring_damping(void *ctx, float damping);

MMBP_API void mmbp_set_spring_damping_linear(void *ctx, int axis,
                                             float damping);
MMBP_API void mmbp_set_spring_damping_angular(void *ctx, int axis,
                                              float damping);
MMBP_API void mmbp_set_bone_rest_info(void *ctx, const float *mat3, int count);
MMBP_API void mmbp_compute_rest_info(const float *bone_local_16,
                                     float *out_mat3x3_9, float *out_offset_3);
MMBP_API int mmbp_step(void *ctx, float frame, float delta, float *out_locs,
                       float *out_quats, int bone_count);
MMBP_API void mmbp_eval_animation(void *ctx, float frame);
MMBP_API void mmbp_solve_ik(void *ctx);
MMBP_API void mmbp_step_physics(void *ctx, float frame, float delta);
MMBP_API void mmbp_sync_physics(void *ctx, float frame, float delta,
                                int frame_count);
MMBP_API void mmbp_get_bone_world_mat4(void *ctx, int idx, float *out_mat4);
MMBP_API int mmbp_get_ik_count(void *ctx);
MMBP_API void mmbp_get_solver_info(void *ctx, float *out);
MMBP_API int mmbp_get_bone_count(void *ctx);
MMBP_API int mmbp_get_bone_name(void *ctx, int idx, char *out, int max_len);

MMBP_API int mmbp_get_all_bone_names(void *ctx, char *names_buf,
                                     int name_buf_size);
MMBP_API int mmbp_get_rigid_body_count(void *ctx);
MMBP_API int mmbp_get_joint_count(void *ctx);
MMBP_API float mmbp_get_fixed_timestep(void *ctx);
MMBP_API const char *mmbp_get_last_error(void *ctx);

MMBP_API int mmbp_get_last_error_code(void *ctx);

MMBP_API int mmbp_get_warning_count(void *ctx);
MMBP_API int mmbp_get_warning(void *ctx, int idx, char *out, int max_len);

MMBP_API void mmbp_set_log_callback(void (*callback)(int level,
                                                     const char *message,
                                                     void *userData),
                                    void *userData);
MMBP_API void mmbp_set_log_level(int level);

MMBP_API int mmbp_bone_has_rigid_body(void *ctx, int bone_idx);

MMBP_API void mmbp_set_ik_loop_count(void *ctx, int bone_idx, int loop_count);
MMBP_API int mmbp_get_ik_loop_count(void *ctx, int bone_idx);

MMBP_API void mmbp_set_ik_max_step_angle(void *ctx, float angle);

MMBP_API int mmbp_reset_to_animation(void *ctx, float frame, float *out_locs,
                                     float *out_quats, int bone_count);

MMBP_API int mmbp_eval_only(void *ctx, float frame, float *out_locs,
                            float *out_quats, int bone_count);

MMBP_API void mmbp_get_active_bone_indices(void *ctx, int *out_indices,
                                           int max_count);

MMBP_API void mmbp_set_sleep_deactivation(void *ctx, int enable);

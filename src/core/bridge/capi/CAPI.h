#pragma once
#include <cstdint>
// C API for Blender/ctypes integration

#ifdef _WIN32
#define MMP_API extern "C" __declspec(dllexport)
#else
#define MMP_API extern "C"
#endif

MMP_API void* mmp_create();
MMP_API void  mmp_destroy(void* ctx);
MMP_API int   mmp_load_pmx(void* ctx, const char* path);
MMP_API int   mmp_load_pmx_mem(void* ctx, const uint8_t* data, int size);
MMP_API int   mmp_load_vmd(void* ctx, const char* path);
MMP_API int   mmp_load_vmd_mem(void* ctx, const uint8_t* data, int size);
MMP_API int   mmp_build_physics(void* ctx);
MMP_API void  mmp_set_scale(void* ctx, float scale);
MMP_API void  mmp_set_gravity(void* ctx, float x, float y, float z);
MMP_API void  mmp_set_solver_iters(void* ctx, int n);
MMP_API void  mmp_set_fixed_timestep(void* ctx, float fps);
MMP_API void  mmp_set_max_substeps(void* ctx, int n);
MMP_API void  mmp_set_bone_rest_info(void* ctx, const float* mat3, int count);
MMP_API void  mmp_set_bone_rest_positions(void* ctx, const float* offset, int count);
MMP_API void  mmp_compute_rest_info(const float* bone_local_16, float* out_mat3x3_9, float* out_offset_3);
MMP_API int   mmp_step(void* ctx, float frame, float delta, float* out_locs, float* out_quats, int bone_count);
MMP_API void  mmp_eval_animation(void* ctx, float frame);
MMP_API void  mmp_solve_ik(void* ctx);
MMP_API void  mmp_step_physics(void* ctx, float frame, float delta);
MMP_API void  mmp_get_bone_world_mat4(void* ctx, int idx, float* out_mat4);
MMP_API int   mmp_get_ik_count(void* ctx);
MMP_API void  mmp_get_solver_info(void* ctx, float* out);
MMP_API int   mmp_get_bone_count(void* ctx);
MMP_API int   mmp_get_bone_name(void* ctx, int idx, char* out, int max_len);
MMP_API int   mmp_get_rigid_body_count(void* ctx);
MMP_API int   mmp_get_joint_count(void* ctx);
MMP_API const char* mmp_get_last_error(void* ctx);

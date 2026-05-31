import ctypes, os, sys

if sys.platform == 'win32':
    DLL_NAME = "mmp_physics.dll"
elif sys.platform == 'darwin':
    DLL_NAME = "mmp_physics.dylib"
else:
    DLL_NAME = "mmp_physics.so"

_cache = None

def load():
    global _cache
    if _cache is not None:
        return _cache
    dll_dir = os.path.dirname(os.path.dirname(__file__))  # parent of core/ = mmp/ root; parent of that = addon root
    # Go up two levels: core/ -> mmp/ -> addon root
    addon_root = os.path.dirname(dll_dir)
    path = os.path.join(addon_root, DLL_NAME)
    if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(addon_root)
    os.environ['PATH'] = addon_root + os.pathsep + os.environ.get('PATH', '')
    lib = ctypes.CDLL(path)

    lib.mmp_create.restype = ctypes.c_void_p
    lib.mmp_destroy.argtypes = [ctypes.c_void_p]
    lib.mmp_load_pmx.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.mmp_load_pmx.restype = ctypes.c_int
    lib.mmp_load_pmx_mem.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.mmp_load_pmx_mem.restype = ctypes.c_int
    lib.mmp_load_vmd.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.mmp_load_vmd.restype = ctypes.c_int
    lib.mmp_load_vmd_mem.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.mmp_load_vmd_mem.restype = ctypes.c_int
    lib.mmp_build_physics.argtypes = [ctypes.c_void_p]
    lib.mmp_build_physics.restype = ctypes.c_int
    lib.mmp_set_gravity.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.mmp_set_solver_iters.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmp_set_fixed_timestep.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmp_set_max_substeps.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmp_set_scale.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmp_set_bone_rest_info.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int]
    lib.mmp_set_bone_rest_positions.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int]
    lib.mmp_compute_rest_info.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
    lib.mmp_step.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
                              ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_int]
    lib.mmp_step.restype = ctypes.c_int
    lib.mmp_eval_animation.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmp_solve_ik.argtypes = [ctypes.c_void_p]
    lib.mmp_step_physics.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.mmp_get_bone_world_mat4.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
    lib.mmp_get_ik_count.argtypes = [ctypes.c_void_p]
    lib.mmp_get_ik_count.restype = ctypes.c_int
    lib.mmp_get_bone_count.argtypes = [ctypes.c_void_p]
    lib.mmp_get_bone_count.restype = ctypes.c_int
    lib.mmp_get_bone_name.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char), ctypes.c_int]
    lib.mmp_get_bone_name.restype = ctypes.c_int
    lib.mmp_get_rigid_body_count.argtypes = [ctypes.c_void_p]
    lib.mmp_get_rigid_body_count.restype = ctypes.c_int
    lib.mmp_get_joint_count.argtypes = [ctypes.c_void_p]
    lib.mmp_get_joint_count.restype = ctypes.c_int
    lib.mmp_get_solver_info.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
    lib.mmp_get_last_error.argtypes = [ctypes.c_void_p]
    lib.mmp_get_last_error.restype = ctypes.c_char_p

    _cache = lib
    return lib

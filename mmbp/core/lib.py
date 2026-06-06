import ctypes, os, sys, logging
from pathlib import Path

logger = logging.getLogger("MMBP")

if sys.platform == "win32":
    DLL_NAME = "mmbp_physics.dll"
elif sys.platform == "darwin":
    DLL_NAME = "mmbp_physics.dylib"
else:
    DLL_NAME = "mmbp_physics.so"

_cache = None
_log_callback_ref = None


_LEVEL_MAP = {0: logging.DEBUG, 1: logging.INFO, 2: logging.WARNING, 3: logging.ERROR}


DEFAULT_LOG_LEVEL = 2


_LOG_CB_TYPE = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p, ctypes.c_void_p)


def _on_log(level, message, user_data):

    try:
        py_level = _LEVEL_MAP.get(level, logging.WARNING)
        logger.log(py_level, message.decode("utf-8", errors="replace"))
    except Exception:
        pass


def _resolve_dll_path():

    addon_root = Path(__file__).resolve().parent.parent.parent
    path = str(addon_root / DLL_NAME)
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(addon_root))
    os.environ["PATH"] = str(addon_root) + os.pathsep + os.environ.get("PATH", "")
    return path


def _load_dll(path):

    try:
        return ctypes.CDLL(path)
    except OSError as e:
        raise OSError(
            f"[MMBP] Failed to load physics library: {DLL_NAME}\n"
            f"  Expected location: {path}\n"
            f"  Platform: {sys.platform}\n"
            f"  Error: {e}\n"
            f"  Ensure {DLL_NAME} exists in the addon root directory."
        ) from e


def _setup_signatures(lib):

    lib.mmbp_create.restype = ctypes.c_void_p
    lib.mmbp_destroy.argtypes = [ctypes.c_void_p]
    lib.mmbp_destroy.restype = None
    lib.mmbp_load_pmx.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.mmbp_load_pmx.restype = ctypes.c_int
    lib.mmbp_load_pmx_mem.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int,
    ]
    lib.mmbp_load_pmx_mem.restype = ctypes.c_int
    lib.mmbp_load_vmd.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.mmbp_load_vmd.restype = ctypes.c_int
    lib.mmbp_load_vmd_mem.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int,
    ]
    lib.mmbp_load_vmd_mem.restype = ctypes.c_int
    lib.mmbp_load_vpd.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.mmbp_load_vpd.restype = ctypes.c_int

    lib.mmbp_merge_vmd_mem.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int,
    ]
    lib.mmbp_merge_vmd_mem.restype = ctypes.c_int
    lib.mmbp_build_physics.argtypes = [ctypes.c_void_p]
    lib.mmbp_build_physics.restype = ctypes.c_int
    lib.mmbp_set_gravity.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
    ]
    lib.mmbp_set_gravity.restype = None
    lib.mmbp_set_solver_iters.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmbp_set_solver_iters.restype = None
    lib.mmbp_set_fixed_timestep.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmbp_set_fixed_timestep.restype = None
    lib.mmbp_set_max_substeps.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmbp_set_max_substeps.restype = None

    lib.mmbp_set_spring_damping.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmbp_set_spring_damping.restype = None
    lib.mmbp_set_scale.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmbp_set_scale.restype = None
    lib.mmbp_set_bone_rest_info.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.mmbp_set_bone_rest_info.restype = None
    lib.mmbp_compute_rest_info.argtypes = [
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.mmbp_compute_rest_info.restype = None
    lib.mmbp_step.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.mmbp_step.restype = ctypes.c_int
    lib.mmbp_eval_animation.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmbp_eval_animation.restype = None
    lib.mmbp_solve_ik.argtypes = [ctypes.c_void_p]
    lib.mmbp_solve_ik.restype = None
    lib.mmbp_step_physics.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.mmbp_step_physics.restype = None

    lib.mmbp_sync_physics.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_int,
    ]
    lib.mmbp_sync_physics.restype = None
    lib.mmbp_get_bone_world_mat4.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.mmbp_get_bone_world_mat4.restype = None
    lib.mmbp_get_ik_count.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_ik_count.restype = ctypes.c_int
    lib.mmbp_get_bone_count.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_bone_count.restype = ctypes.c_int
    lib.mmbp_get_bone_name.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_char),
        ctypes.c_int,
    ]
    lib.mmbp_get_bone_name.restype = ctypes.c_int

    lib.mmbp_get_all_bone_names.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_char),
        ctypes.c_int,
    ]
    lib.mmbp_get_all_bone_names.restype = ctypes.c_int
    lib.mmbp_get_rigid_body_count.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_rigid_body_count.restype = ctypes.c_int
    lib.mmbp_get_joint_count.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_joint_count.restype = ctypes.c_int
    lib.mmbp_get_solver_info.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.mmbp_get_solver_info.restype = None
    lib.mmbp_get_fixed_timestep.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_fixed_timestep.restype = ctypes.c_float
    lib.mmbp_get_last_error.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_last_error.restype = ctypes.c_char_p
    lib.mmbp_get_last_error_code.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_last_error_code.restype = ctypes.c_int

    lib.mmbp_get_warning_count.argtypes = [ctypes.c_void_p]
    lib.mmbp_get_warning_count.restype = ctypes.c_int
    lib.mmbp_get_warning.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_char),
        ctypes.c_int,
    ]
    lib.mmbp_get_warning.restype = ctypes.c_int

    lib.mmbp_set_log_callback.argtypes = [_LOG_CB_TYPE, ctypes.c_void_p]
    lib.mmbp_set_log_callback.restype = None
    lib.mmbp_set_log_level.argtypes = [ctypes.c_int]
    lib.mmbp_set_log_level.restype = None

    lib.mmbp_bone_has_rigid_body.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmbp_bone_has_rigid_body.restype = ctypes.c_int

    lib.mmbp_set_ik_loop_count.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.mmbp_set_ik_loop_count.restype = None
    lib.mmbp_get_ik_loop_count.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmbp_get_ik_loop_count.restype = ctypes.c_int

    lib.mmbp_set_sleep_deactivation.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mmbp_set_sleep_deactivation.restype = None

    lib.mmbp_set_spring_damping_linear.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_float,
    ]
    lib.mmbp_set_spring_damping_linear.restype = None
    lib.mmbp_set_spring_damping_angular.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_float,
    ]
    lib.mmbp_set_spring_damping_angular.restype = None

    lib.mmbp_set_ik_max_step_angle.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.mmbp_set_ik_max_step_angle.restype = None

    lib.mmbp_reset_to_animation.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.mmbp_reset_to_animation.restype = ctypes.c_int
    lib.mmbp_eval_only.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.mmbp_eval_only.restype = ctypes.c_int

    _all_names = [
        "mmbp_create",
        "mmbp_destroy",
        "mmbp_load_pmx",
        "mmbp_load_pmx_mem",
        "mmbp_load_vmd",
        "mmbp_load_vmd_mem",
        "mmbp_load_vpd",
        "mmbp_merge_vmd_mem",
        "mmbp_build_physics",
        "mmbp_set_gravity",
        "mmbp_set_solver_iters",
        "mmbp_set_fixed_timestep",
        "mmbp_set_max_substeps",
        "mmbp_set_spring_damping",
        "mmbp_set_spring_damping_linear",
        "mmbp_set_spring_damping_angular",
        "mmbp_set_scale",
        "mmbp_set_bone_rest_info",
        "mmbp_compute_rest_info",
        "mmbp_step",
        "mmbp_eval_animation",
        "mmbp_solve_ik",
        "mmbp_step_physics",
        "mmbp_sync_physics",
        "mmbp_get_bone_world_mat4",
        "mmbp_get_ik_count",
        "mmbp_get_bone_count",
        "mmbp_get_bone_name",
        "mmbp_get_all_bone_names",
        "mmbp_get_rigid_body_count",
        "mmbp_get_joint_count",
        "mmbp_get_solver_info",
        "mmbp_get_fixed_timestep",
        "mmbp_get_last_error",
        "mmbp_get_last_error_code",
        "mmbp_get_warning_count",
        "mmbp_get_warning",
        "mmbp_set_log_callback",
        "mmbp_set_log_level",
        "mmbp_bone_has_rigid_body",
        "mmbp_set_ik_loop_count",
        "mmbp_get_ik_loop_count",
        "mmbp_set_ik_max_step_angle",
        "mmbp_set_sleep_deactivation",
        "mmbp_reset_to_animation",
        "mmbp_eval_only",
    ]
    for name in _all_names:
        func = getattr(lib, name, None)
        if func is None:
            logger.error("[MMBP] C API function '%s' not found in DLL", name)
            continue
        has_argtypes = hasattr(func, 'argtypes') and func.argtypes
        if not has_argtypes and func.restype == ctypes.c_int:
            logger.warning("[MMBP] C API function '%s' not registered in _setup_signatures", name)


def _setup_logging(lib):

    global _log_callback_ref
    _log_callback_ref = _LOG_CB_TYPE(_on_log)
    lib.mmbp_set_log_callback(_log_callback_ref, None)
    lib.mmbp_set_log_level(DEFAULT_LOG_LEVEL)


def dll_path():

    return _resolve_dll_path()


def load():
    global _cache
    if _cache is not None:
        return _cache
    path = _resolve_dll_path()
    lib = _load_dll(path)
    _setup_signatures(lib)
    _setup_logging(lib)
    _cache = lib
    return lib

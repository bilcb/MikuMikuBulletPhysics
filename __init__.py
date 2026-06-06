bl_info = {
    "name": "MikuMikuBulletPhysics",
    "version": (0, 0, 1),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > MMBP",
    "category": "Animation",
    "doc_url": "https://github.com/bilcb/MikuMikuBulletPhysics",
    "tracker_url": "https://github.com/bilcb/MikuMikuBulletPhysics/issues",
}

import bpy
from .mmbp.core import state, loop
from .mmbp.core.state import DEFAULT_GRAVITY, DEFAULT_SOLVER_ITERATIONS, DEFAULT_FIXED_TIMESTEP
from .mmbp.core.loop import register_frame_handler, unregister_frame_handler
from .mmbp.bridge import ui
from .mmbp.ops import start, load

_manager = state.get_manager()

CLASSES = [
    *start.CLASSES,
    *load.CLASSES,
    *ui.CLASSES,
]


def _verify_mmd_root() -> bool:
    has_mmd_tools = 'mmd_tools' in bpy.context.preferences.addons
    if not has_mmd_tools:
        print("[MMBP] WARNING: 'mmd_tools' addon is not installed or "
              "not enabled. MMBP requires mmd_tools for bone name matching "
              "(mmd_bone.name_j / bone_id). Install from "
              "https://github.com/powroupi/blender_mmd_tools")
        return False
    return True


def register():
    _verify_mmd_root()
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.app.handlers.load_post.append(_on_load_post)
    bpy.app.handlers.save_pre.append(_on_save_pre)
    register_frame_handler()
    bpy.types.Scene.mmbp_gravity = bpy.props.FloatVectorProperty(
        name="Gravity",
        size=3,
        default=DEFAULT_GRAVITY,
        subtype='ACCELERATION',
        description="Gravity vector in MMD space (dm/s²)"
    )
    bpy.types.Scene.mmbp_solver_iterations = bpy.props.IntProperty(
        name="Solver Iterations",
        default=DEFAULT_SOLVER_ITERATIONS,
        min=1, max=100,
        description="Constraint solver iteration count"
    )
    bpy.types.Scene.mmbp_fixed_timestep = bpy.props.EnumProperty(
        name="Physics Rate",
        items=[
            ('30', '30 Hz', '0.0333s step'),
            ('60', '60 Hz', '0.0167s step'),
            ('90', '90 Hz', '0.0111s step'),
            ('120', '120 Hz', '0.0083s step (default)'),
            ('240', '240 Hz', '0.0042s step'),
        ],
        default='60',
        description="Physics simulation frequency — higher = smoother but more CPU"
    )
    bpy.types.Scene.mmbp_scale = bpy.props.FloatProperty(
        name="Scale",
        default=0.0,
        min=0.0, max=1.0, precision=4,
        description="PMX→Blender scale override (0 = auto-detect from mmd_tools)"
    )
    bpy.types.Scene.mmbp_max_substeps = bpy.props.IntProperty(
        name="Max Substeps",
        default=6,
        min=1, max=100,
        description="Maximum physics substeps per frame (prevents spiral-of-death)"
    )
    bpy.types.Scene.mmbp_spring_damping = bpy.props.FloatProperty(
        name="Spring Damping",
        default=0.3,
        min=0.0, max=1.0, precision=3,
        description="Joint spring damping factor (0=no damping, 1=critical)"
    )
    bpy.types.Scene.mmbp_write_interval = bpy.props.IntProperty(
        name="Write Interval",
        default=1,
        min=1, max=60,
        description="Write physics results to Blender bones every N substeps (1=every step, higher=smoother but less responsive)"
    )
    bpy.types.Scene.mmbp_log_level = bpy.props.EnumProperty(
        name="Log Level",
        items=[
            ('0', "DEBUG", "All messages"),
            ('1', "INFO", "Info and above"),
            ('2', "WARN", "Warnings and errors only"),
            ('3', "ERROR", "Errors only"),
        ],
        default='2',
        description="C++ engine log verbosity"
    )


def unregister():
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)
    if _on_save_pre in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.remove(_on_save_pre)
    unregister_frame_handler()
    _manager.shutdown_all()
    for prop in ("mmbp_gravity", "mmbp_solver_iterations", "mmbp_fixed_timestep", "mmbp_scale", "mmbp_max_substeps", "mmbp_log_level", "mmbp_spring_damping", "mmbp_write_interval"):
        if hasattr(bpy.types.Scene, prop):
            delattr(bpy.types.Scene, prop)
    for cls in reversed(CLASSES):
        try:
            bpy.utils.unregister_class(cls)
        except Exception as e:
            print(f"[MMBP] Failed to unregister {cls.__name__}: {e}")


@bpy.app.handlers.persistent
def _on_load_post(dummy):
    _manager.shutdown_all()


@bpy.app.handlers.persistent
def _on_save_pre(dummy):
    _manager.shutdown_all()

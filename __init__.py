bl_info = {
    "name": "MikuMikuBulletPhysics",
    "version": (0, 0, 1),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > MMP",
    "category": "Animation",
}

import bpy
from .mmp.core import dll, state, loop
from .mmp.bridge import ui
from .mmp.ops import start, load

_manager = state.get_manager()

CLASSES = [
    *start.CLASSES,
    *load.CLASSES,
    *ui.CLASSES,
]


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.app.handlers.load_post.append(_on_load_post)
    bpy.app.handlers.save_pre.append(_on_save_pre)


def unregister():
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)
    if _on_save_pre in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.remove(_on_save_pre)
    _manager.shutdown_all()
    for cls in reversed(CLASSES):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass


@bpy.app.handlers.persistent
def _on_load_post(dummy):
    _manager.shutdown_all()


@bpy.app.handlers.persistent
def _on_save_pre(dummy):
    _manager.shutdown_all()

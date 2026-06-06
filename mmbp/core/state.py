from __future__ import annotations

import bpy, ctypes
from typing import Optional, Dict
from .bonemap import BoneMapper
from . import lib as _lib


def resolve_armature(context: bpy.types.Context) -> Optional[bpy.types.Object]:

    arm = context.active_object
    if arm and arm.type == "ARMATURE":
        return arm
    armatures = [obj for obj in context.blend_data.objects if obj.type == "ARMATURE"]
    if len(armatures) == 1:
        return armatures[0]
    return None


DEFAULT_SCALE = 0.08
DEFAULT_GRAVITY = (0.0, -9.8, 0.0)
DEFAULT_SOLVER_ITERATIONS = 10
DEFAULT_FIXED_TIMESTEP = 1.0 / 60.0
DEFAULT_MAX_SUBSTEPS = 6


def get_physics_timestep(scene):

    try:
        hz = int(scene.mmbp_fixed_timestep)
        return 1.0 / hz if hz > 0 else 1.0 / 120.0
    except (ValueError, TypeError, AttributeError):
        return 1.0 / 120.0


def get_physics_gravity(scene):

    g = getattr(scene, "mmbp_gravity", DEFAULT_GRAVITY)
    return (g[0] * 10.0, g[1] * 10.0, g[2] * 10.0)


class State:
    ctx: Optional[int]
    armature: Optional[bpy.types.Object]
    running: bool
    bone_count: int
    last_time: float
    accumulator: float
    locs: Optional[ctypes.Array[ctypes.c_float]]
    quats: Optional[ctypes.Array[ctypes.c_float]]
    bone_mapper: Optional["BoneMapper"]

    def __init__(self, armature: Optional[bpy.types.Object] = None):
        self.lib = _lib.load()
        self.ctx = self.lib.mmbp_create()
        if not self.ctx:
            raise RuntimeError("mmbp_create failed — DLL initialization error")
        self.armature = armature
        self.running = False
        self.bone_count = 0
        self.last_time = 0.0
        self.accumulator = 0.0
        self.locs = None
        self.quats = None
        self.bone_mapper = None
        self._disable_count = 0
        self._undo_disabled = False
        self._saved_action = None
        self._saved_nla_tracks: list = []
        self._saved_rb_world = None
        self._saved_mmd_other: list = []
        self._cached_scene = None

    def _invoke_checked(self, func, *args, check_error=True):

        result = func(self.ctx, *args)
        if check_error and not result:
            err = self.lib.mmbp_get_last_error(self.ctx)
            if err:
                raise RuntimeError(f"MMBP: {err.decode('utf-8', errors='replace')}")
        return result

    def destroy(self) -> None:
        if self.ctx:
            self.lib.mmbp_destroy(self.ctx)
            self.ctx = None

    @staticmethod
    def _detect_scale(armature_obj, scene=None):

        if (
            scene is not None
            and hasattr(scene, "mmbp_scale")
            and scene.mmbp_scale > 0.0001
        ):
            return scene.mmbp_scale

        try:
            root = armature_obj.parent
            if root and getattr(root, "mmd_type", "") == "ROOT":
                scale = root.empty_display_size * 0.2
                if scale > 0.0001:
                    return scale
        except (AttributeError, ReferenceError) as e:
            print(f"[MMBP] _detect_scale: mmd_tools detection failed: {e}")
        return DEFAULT_SCALE

    def load_model(self, pmx_path: str, armature_obj: bpy.types.Object) -> None:
        import os

        if not pmx_path or not os.path.isfile(pmx_path):
            raise RuntimeError(f"PMX file not found: {pmx_path}")
        if armature_obj is None or armature_obj.type != "ARMATURE":
            raise RuntimeError(
                f"Expected ARMATURE object, got {getattr(armature_obj, 'type', None)}"
            )
        self.running = False
        self.restore_all()

        scene = self._find_scene()
        scale = self._detect_scale(armature_obj, scene)
        print(f"[MMBP] load_model: scale={scale:.4f} path={pmx_path}")

        self.lib.mmbp_set_scale(self.ctx, scale)

        gravity = get_physics_gravity(scene)
        solver_iters = getattr(
            scene, "mmbp_solver_iterations", DEFAULT_SOLVER_ITERATIONS
        )
        fixed_step = get_physics_timestep(scene)
        max_substeps = getattr(scene, "mmbp_max_substeps", DEFAULT_MAX_SUBSTEPS)
        self.lib.mmbp_set_gravity(self.ctx, *gravity)
        self.lib.mmbp_set_solver_iters(self.ctx, solver_iters)
        self.lib.mmbp_set_fixed_timestep(self.ctx, fixed_step)
        self.lib.mmbp_set_max_substeps(self.ctx, max_substeps)

        try:
            self._invoke_checked(self.lib.mmbp_load_pmx, pmx_path.encode("utf-8"))
            self._invoke_checked(self.lib.mmbp_build_physics)
        except Exception:

            self.armature = None
            self.bone_count = 0
            self.bone_mapper = None
            self.locs = None
            self.quats = None
            raise
        self.armature = armature_obj
        self.bone_count = self.lib.mmbp_get_bone_count(self.ctx)
        self.bone_mapper = BoneMapper(self.bone_count)
        self._alloc_buffers()
        self.bone_mapper.init_mapping(
            self.lib, self.ctx, self.armature, self.bone_count
        )
        print(
            f"[MMBP] load_model done: bones={self.bone_count} rb={self.lib.mmbp_get_rigid_body_count(self.ctx)} jt={self.lib.mmbp_get_joint_count(self.ctx)}"
        )

    def load_pose(self, vpd_path: str) -> None:
        import os

        if not vpd_path or not os.path.isfile(vpd_path):
            raise RuntimeError(f"VPD file not found: {vpd_path}")
        ok = self._invoke_checked(self.lib.mmbp_load_vpd, vpd_path.encode("utf-8"))
        if ok:
            print(f"[MMBP] VPD pose loaded: {vpd_path}")
        else:
            err = self.lib.mmbp_get_last_error(self.ctx)
            msg = err.decode("utf-8", errors="replace") if err else "unknown error"
            print(f"[MMBP] VPD load failed: {msg}")
            raise RuntimeError(f"VPD load failed: {msg}")

    def load_motion(self, vmd_path: str) -> None:
        import os

        if not vmd_path or not os.path.isfile(vmd_path):
            raise RuntimeError(f"VMD file not found: {vmd_path}")
        self._invoke_checked(self.lib.mmbp_load_vmd, vmd_path.encode("utf-8"))

        warn_count = self.lib.mmbp_get_warning_count(self.ctx)
        if warn_count > 0:
            buf = ctypes.create_string_buffer(256)
            for i in range(warn_count):
                if self.lib.mmbp_get_warning(self.ctx, i, buf, 256):
                    print(
                        f"[MMBP] VMD warning: {buf.value.decode('utf-8', errors='replace')}"
                    )

    def _alloc_buffers(self):
        n = self.bone_count
        self.locs = (ctypes.c_float * (n * 3))()
        self.quats = (ctypes.c_float * (n * 4))()

    def write_to_blender(self) -> None:
        if self.bone_mapper:
            self.bone_mapper.write_to_blender(self.locs, self.quats)

    def reset_to_animation(self, frame: float) -> None:

        if not self.ctx or self.locs is None or self.quats is None:
            return
        self.lib.mmbp_reset_to_animation(
            self.ctx, frame, self.locs, self.quats, self.bone_count
        )
        self.write_to_blender()

    def eval_pose_only(self, frame: float) -> None:

        if not self.ctx or self.locs is None or self.quats is None:
            return
        self.lib.mmbp_eval_only(self.ctx, frame, self.locs, self.quats, self.bone_count)
        self.write_to_blender()

    def _find_scene(self):

        if self._cached_scene is not None:
            try:
                if self.armature and self.armature.name in self._cached_scene.objects:
                    return self._cached_scene
            except ReferenceError:
                self._cached_scene = None
        if self.armature:
            for s in bpy.data.scenes:
                if self.armature.name in s.objects:
                    self._cached_scene = s
                    return s
        return bpy.context.scene

    _CONFLICTING_CONSTRAINT_TYPES = {
        "IK",
        "COPY_TRANSFORMS",
        "COPY_ROTATION",
        "COPY_LOCATION",
        "DAMPED_TRACK",
        "TRACK_TO",
        "LOCKED_TRACK",
        "STRETCH_TO",
        "CHILD_OF",
    }

    def disable_all(self):

        arm = self.armature
        if arm is None:
            return

        if self._disable_count == 0:
            try:

                if bpy.context.preferences.edit.use_global_undo:
                    bpy.context.preferences.edit.use_global_undo = False
                    self._undo_disabled = True

                if arm.animation_data:
                    self._saved_action = arm.animation_data.action
                    arm.animation_data.action = None
                    for track in arm.animation_data.nla_tracks:
                        self._saved_nla_tracks.append(track)
                        track.mute = True

                scene = self._find_scene()
                if scene.rigidbody_world:
                    self._saved_rb_world = scene.rigidbody_world.enabled
                    scene.rigidbody_world.enabled = False

                for pb in arm.pose.bones:
                    for con in pb.constraints:
                        is_mmd = con.name.lower().startswith(
                            "mmd_"
                        ) or con.name.lower().startswith("mmd ")
                        is_conflicting = con.type in self._CONFLICTING_CONSTRAINT_TYPES
                        if not is_mmd and not is_conflicting:
                            continue
                        try:
                            self._saved_mmd_other.append((con, pb))
                            con.mute = True
                        except ReferenceError:
                            pass
            except Exception as e:
                print(f"[MMBP] disable_all partial failure: {e}")
                import traceback

                traceback.print_exc()

        self._disable_count += 1

    def restore_all(self):

        try:
            arm = self.armature
        except ReferenceError:

            self._saved_action = None
            self._saved_nla_tracks.clear()
            self._saved_rb_world = None
            self._saved_mmd_other.clear()
            self._undo_disabled = False
            self._disable_count = 0
            self.destroy()
            return

        self._disable_count = max(0, self._disable_count - 1)
        if self._disable_count > 0:
            return

        if arm and arm.animation_data:
            if self._saved_action is not None:
                arm.animation_data.action = self._saved_action
            for track in self._saved_nla_tracks:
                try:
                    track.mute = False
                except ReferenceError:
                    pass
        self._saved_action = None
        self._saved_nla_tracks.clear()

        scene = self._find_scene()
        if self._saved_rb_world is not None and scene.rigidbody_world:
            scene.rigidbody_world.enabled = self._saved_rb_world
        self._saved_rb_world = None

        for con, pb in self._saved_mmd_other:
            try:
                con.mute = False
            except ReferenceError:
                pass
        self._saved_mmd_other.clear()

        if self._undo_disabled:
            bpy.context.preferences.edit.use_global_undo = True
            self._undo_disabled = False


class StateManager:

    def __init__(self):
        self._states: Dict[int, State] = {}
        self._name_index: Dict[str, int] = {}

    def get_or_create(self, armature) -> State:
        ptr = armature.as_pointer()
        if ptr not in self._states or self._states[ptr].ctx is None:
            self._states[ptr] = State(armature)
            self._name_index[armature.name] = ptr
        else:

            self._states[ptr].armature = armature
            self._name_index[armature.name] = ptr
        return self._states[ptr]

    def get(self, armature_name: str) -> Optional[State]:
        ptr = self._name_index.get(armature_name)
        if ptr is not None:
            return self._states.get(ptr)
        return None

    def get_by_pointer(self, pointer: int) -> Optional[State]:
        return self._states.get(pointer)

    def remove(self, armature_name: str):
        ptr = self._name_index.pop(armature_name, None)
        if ptr is not None:
            st = self._states.pop(ptr, None)
            if st:
                if st.running:
                    st.running = False
                    st.restore_all()
                st.destroy()

    def shutdown_all(self):
        for st in list(self._states.values()):
            if st.running:
                st.running = False
            try:
                st.restore_all()
            except Exception as e:
                print(f"[MMBP] restore_all failed during shutdown: {e}")
            st.destroy()
        self._states.clear()
        self._name_index.clear()

    def get_active_names(self) -> list:
        names = []
        for st in self._states.values():
            try:
                if st.armature is not None:
                    names.append(st.armature.name)
            except ReferenceError:
                pass
        return names

    def has_running(self) -> bool:

        return any(st.running for st in self._states.values() if st)


_manager = StateManager()


def get_manager():
    return _manager

from __future__ import annotations

import bpy, ctypes, time
from typing import Optional, Dict
from . import dll


def resolve_armature(context: bpy.types.Context) -> Optional[bpy.types.Object]:
    """Resolve armature from context: active object → single armature fallback."""
    arm = context.active_object
    if arm and arm.type == 'ARMATURE':
        return arm
    armatures = [obj for obj in context.blend_data.objects if obj.type == 'ARMATURE']
    if len(armatures) == 1:
        return armatures[0]
    return None

DEFAULT_SCALE = 0.08
DEFAULT_GRAVITY = (0.0, -9.8, 0.0)
DEFAULT_SOLVER_ITERATIONS = 20
DEFAULT_FIXED_TIMESTEP = 1.0 / 120.0
DEFAULT_MAX_SUBSTEPS = 10

class State:
    ctx: Optional[int]
    armature: Optional[bpy.types.Object]
    running: bool
    bone_count: int
    last_time: float
    accumulator: float
    locs: Optional[ctypes.Array[ctypes.c_float]]
    quats: Optional[ctypes.Array[ctypes.c_float]]
    _bone_map: Dict[int, bpy.types.Bone]

    def __init__(self, armature: Optional[bpy.types.Object] = None):
        self.lib = dll.load()
        self.ctx = self.lib.mmp_create()
        self.armature = armature
        self.running = False
        self.bone_count = 0
        self.last_time = 0.0
        self.accumulator = 0.0
        self.locs = None
        self.quats = None
        self._bone_map = {}
        self._rotation_mode_set = False
        self._undo_disabled = False
        self._saved_action = None
        self._saved_rb_world = None
        self._saved_use_prop_driver = None
        self._saved_constraints: list = []
        self._saved_ik: list = []
        self._saved_mmd_other: list = []

    def destroy(self) -> None:
        if self.ctx:
            self.lib.mmp_destroy(self.ctx)
            self.ctx = None

    @staticmethod
    def _detect_scale(armature_obj):
        root = armature_obj.parent
        if root and getattr(root, 'mmd_type', '') == 'ROOT':
            scale = root.empty_display_size * 0.2
            if scale > 0.0001:
                return scale
        return DEFAULT_SCALE

    def load_model(self, pmx_path: str, armature_obj: bpy.types.Object) -> None:
        import os
        if not pmx_path or not os.path.isfile(pmx_path):
            raise ValueError(f"PMX file not found: {pmx_path}")
        if armature_obj is None or armature_obj.type != 'ARMATURE':
            raise ValueError(f"Expected ARMATURE object, got {getattr(armature_obj, 'type', None)}")
        self.running = False  # reset stale flag
        self.restore_all()
        scale = self._detect_scale(armature_obj)
        print(f"[MMP] load_model: scale={scale:.4f} path={pmx_path}")
        self.lib.mmp_set_scale(self.ctx, scale)
        ok = self.lib.mmp_load_pmx(self.ctx, pmx_path.encode('utf-8'))
        if not ok:
            err = self.lib.mmp_get_last_error(self.ctx)
            raise RuntimeError(err.decode() if err else "Failed to load PMX")
        # Set physics config BEFORE buildPhysics so World::create() picks up these values
        self.lib.mmp_set_gravity(self.ctx, *DEFAULT_GRAVITY)
        self.lib.mmp_set_solver_iters(self.ctx, DEFAULT_SOLVER_ITERATIONS)
        self.lib.mmp_set_fixed_timestep(self.ctx, DEFAULT_FIXED_TIMESTEP)
        self.lib.mmp_set_max_substeps(self.ctx, DEFAULT_MAX_SUBSTEPS)
        ok = self.lib.mmp_build_physics(self.ctx)
        if not ok:
            err = self.lib.mmp_get_last_error(self.ctx)
            raise RuntimeError(err.decode() if err else "Failed to build physics")
        self.armature = armature_obj
        self.bone_count = self.lib.mmp_get_bone_count(self.ctx)
        self._bone_map = {}
        self._rotation_mode_set = False
        self._alloc_buffers()
        self._set_bone_rest_info()
        print(f"[MMP] load_model done: bones={self.bone_count} rb={self.lib.mmp_get_rigid_body_count(self.ctx)} jt={self.lib.mmp_get_joint_count(self.ctx)}")

    def load_motion(self, vmd_path: str) -> None:
        import os
        if not vmd_path or not os.path.isfile(vmd_path):
            raise ValueError(f"VMD file not found: {vmd_path}")
        ok = self.lib.mmp_load_vmd(self.ctx, vmd_path.encode('utf-8'))
        if not ok:
            err = self.lib.mmp_get_last_error(self.ctx)
            raise RuntimeError(err.decode() if err else "Failed to load VMD")

    def _alloc_buffers(self):
        n = self.bone_count
        self.locs = (ctypes.c_float * (n * 3))()
        self.quats = (ctypes.c_float * (n * 4))()

    def _match_bone(self, pmx_idx: int, pmx_name: str, bones, name_j_map: dict, bone_id_map: dict):
        """Match a PMX bone index to a Blender bone using 4 strategies."""
        # Strategy 0: bone_id (most reliable)
        matched = bone_id_map.get(pmx_idx)
        # Strategy 1: mmd_bone.name_j
        if matched is None:
            matched = name_j_map.get(pmx_name)
        # Strategy 2: Blender bone name
        if matched is None and pmx_name in bones:
            matched = bones[pmx_name]
        # Strategy 3: case-insensitive
        if matched is None:
            pmx_lower = pmx_name.lower()
            for b in bones.values() if hasattr(bones, 'values') else bones:
                if b.name.lower() == pmx_lower:
                    matched = b
                    break
        return matched

    def _set_bone_rest_info(self):
        data_bones = self.armature.data.bones
        pose_bones = self.armature.pose.bones
        mat3_array = (ctypes.c_float * (self.bone_count * 9))()
        offset_array = (ctypes.c_float * (self.bone_count * 3))()
        name_buf = ctypes.create_string_buffer(256)
        names = [None] * self.bone_count
        for pmx_idx in range(self.bone_count):
            self.lib.mmp_get_bone_name(self.ctx, pmx_idx, name_buf, 256)
            names[pmx_idx] = name_buf.value.decode('utf-8', errors='replace').rstrip('\x00 ')
        name_j_to_bone = {}
        for pb in pose_bones:
            mmd_bone = getattr(pb, 'mmd_bone', None)
            if mmd_bone:
                n = getattr(mmd_bone, 'name_j', '')
                if n:
                    name_j_to_bone[n] = pb.bone
        for pmx_idx, pmx_name in enumerate(names):
            bl_bone = self._match_bone(pmx_idx, pmx_name, data_bones, name_j_to_bone, {})
            if bl_bone:
                # Delegate matrix math to DLL (C++) via CAPI
                ml = bl_bone.matrix_local
                bone_local = (ctypes.c_float * 16)(
                    ml[0][0], ml[0][1], ml[0][2], ml[0][3],
                    ml[1][0], ml[1][1], ml[1][2], ml[1][3],
                    ml[2][0], ml[2][1], ml[2][2], ml[2][3],
                    ml[3][0], ml[3][1], ml[3][2], ml[3][3])
                out_mat = (ctypes.c_float * 9)()
                out_off = (ctypes.c_float * 3)()
                self.lib.mmp_compute_rest_info(bone_local, out_mat, out_off)
                base = pmx_idx * 9
                for i in range(9):
                    mat3_array[base + i] = out_mat[i]
                base3 = pmx_idx * 3
                offset_array[base3] = out_off[0]
                offset_array[base3 + 1] = out_off[1]
                offset_array[base3 + 2] = out_off[2]
            else:
                base = pmx_idx * 9
                mat3_array[base] = 1
                mat3_array[base + 4] = 1
                mat3_array[base + 8] = 1
        self.lib.mmp_set_bone_rest_info(self.ctx, mat3_array, self.bone_count)
        self.lib.mmp_set_bone_rest_positions(self.ctx, offset_array, self.bone_count)

    def _build_bone_map(self):
        if self._bone_map:
            return
        bones = self.armature.pose.bones
        name_buf = ctypes.create_string_buffer(256)
        names = [None] * self.bone_count
        for pmx_idx in range(self.bone_count):
            self.lib.mmp_get_bone_name(self.ctx, pmx_idx, name_buf, 256)
            names[pmx_idx] = name_buf.value.decode('utf-8', errors='replace').rstrip('\x00 ')
        name_j_map = {}
        bone_id_map = {}
        for pb in bones:
            mmd = getattr(pb, 'mmd_bone', None)
            if mmd:
                n = getattr(mmd, 'name_j', '')
                if n:
                    name_j_map[n] = pb
                bid = getattr(mmd, 'bone_id', -1)
                if bid >= 0:
                    bone_id_map[bid] = pb
        for pmx_idx, pmx_name in enumerate(names):
            matched = self._match_bone(pmx_idx, pmx_name, bones, name_j_map, bone_id_map)
            if matched:
                self._bone_map[pmx_idx] = matched
                matched.rotation_mode = 'QUATERNION'
        self._rotation_mode_set = True

    def write_to_blender(self) -> None:
        import math
        self._build_bone_map()
        try:
            for pmx_idx, bone in self._bone_map.items():
                i3 = pmx_idx * 3
                i4 = pmx_idx * 4
                loc = (
                    self.locs[i3] if math.isfinite(self.locs[i3]) else 0.0,
                    self.locs[i3 + 1] if math.isfinite(self.locs[i3 + 1]) else 0.0,
                    self.locs[i3 + 2] if math.isfinite(self.locs[i3 + 2]) else 0.0,
                )
                w = self.quats[i4] if math.isfinite(self.quats[i4]) else 1.0
                x = self.quats[i4 + 1] if math.isfinite(self.quats[i4 + 1]) else 0.0
                y = self.quats[i4 + 2] if math.isfinite(self.quats[i4 + 2]) else 0.0
                z = self.quats[i4 + 3] if math.isfinite(self.quats[i4 + 3]) else 0.0
                bone.location = loc
                bone.rotation_quaternion = (w, x, y, z)
        except ReferenceError:
            pass  # armature deleted

    def _find_scene(self):
        """Find the scene containing this armature."""
        if self.armature and self.armature.users_scene:
            for s in bpy.data.scenes:
                if self.armature.name in s.objects:
                    return s
        return bpy.context.scene

    def disable_all(self):
        arm = self.armature
        # Disable global undo to prevent undo bloat (120Hz writes = 120 undo steps/s)
        if bpy.context.preferences.edit.use_global_undo:
            bpy.context.preferences.edit.use_global_undo = False
            self._undo_disabled = True
        # Save and disable Action + NLA tracks
        if arm and arm.animation_data:
            self._saved_action = arm.animation_data.action
            arm.animation_data.action = None
            self._saved_nla_tracks = []
            for track in arm.animation_data.nla_tracks:
                self._saved_nla_tracks.append(track)
                track.mute = True
        scene = self._find_scene()
        if scene.rigidbody_world:
            self._saved_rb_world = scene.rigidbody_world.enabled
            scene.rigidbody_world.enabled = False
        for pb in arm.pose.bones:
            for con in pb.constraints:
                # Mute mmd constraints and IK/COPY_TRANSFORMS that may conflict
                is_mmd = 'mmd' in con.name.lower()
                is_conflicting = con.type in ('IK', 'COPY_TRANSFORMS')
                if not is_mmd and not is_conflicting:
                    continue
                try:
                    if con.type == 'COPY_TRANSFORMS':
                        self._saved_constraints.append((con, pb))
                        con.mute = True
                    elif con.type == 'IK':
                        self._saved_ik.append((con, pb))
                        con.mute = True
                    else:
                        self._saved_mmd_other.append((con, pb))
                        con.mute = True
                except ReferenceError:
                    pass

    def restore_all(self):
        arm = self.armature
        if arm and arm.animation_data:
            if self._saved_action is not None:
                arm.animation_data.action = self._saved_action
            for track in getattr(self, '_saved_nla_tracks', []):
                try:
                    track.mute = False
                except ReferenceError:
                    pass
        self._saved_action = None
        scene = self._find_scene()
        if self._saved_rb_world is not None and scene.rigidbody_world:
            scene.rigidbody_world.enabled = self._saved_rb_world
        self._saved_rb_world = None
        for items in (self._saved_constraints, self._saved_ik, self._saved_mmd_other):
            for con, pb in items:
                try:
                    con.mute = False
                except ReferenceError:
                    pass
        self._saved_constraints.clear()
        self._saved_ik.clear()
        self._saved_mmd_other.clear()
        # Restore global undo if we disabled it
        if self._undo_disabled:
            bpy.context.preferences.edit.use_global_undo = True
            self._undo_disabled = False


class StateManager:
    def __init__(self):
        self._states = {}

    def get_or_create(self, armature):
        key = armature.name
        if key not in self._states or self._states[key].ctx is None:
            self._states[key] = State(armature)
        return self._states[key]

    def get(self, armature_name):
        return self._states.get(armature_name)

    def remove(self, armature_name):
        st = self._states.pop(armature_name, None)
        if st:
            st.destroy()

    def shutdown_all(self):
        for st in list(self._states.values()):
            if st.running:
                st.running = False
                st.restore_all()
            st.destroy()
        self._states.clear()

    def get_active_names(self):
        return list(self._states.keys())


_manager = StateManager()

def get_manager():
    return _manager

from __future__ import annotations
import re, math
from typing import Dict, Optional
import bpy, ctypes

_BASE_NAME_RE = re.compile(r"\.\d+$")


class BoneMapper:

    def __init__(self, bone_count: int):
        self.bone_count = bone_count
        self._bone_names_cache: Optional[list] = None
        self.bone_map: Dict[int, bpy.types.PoseBone] = {}
        self._last_match_level: Dict[int, int] = {}

        self._last_locs = None
        self._last_quats = None

    def get_bone_names(self, lib, ctx) -> list:
        if self._bone_names_cache is not None:
            return self._bone_names_cache

        name_buf_size = 256
        buf = (ctypes.c_char * (self.bone_count * name_buf_size))()
        count = lib.mmbp_get_all_bone_names(ctx, buf, name_buf_size)
        names = [None] * self.bone_count
        for i in range(min(count, self.bone_count)):
            raw = bytes(buf[i * name_buf_size : (i + 1) * name_buf_size])
            null_pos = raw.find(b"\x00")
            if null_pos >= 0:
                raw = raw[:null_pos]
            names[i] = raw.decode("utf-8", errors="replace")

        for i in range(self.bone_count):
            if names[i] is None:
                names[i] = ""
        self._bone_names_cache = names
        return names

    def invalidate_cache(self):
        self._bone_names_cache = None

    @staticmethod
    def match_bone(
        pmx_idx: int,
        pmx_name: str,
        pose_bones,
        name_j_map: dict,
        bone_id_map: dict,
        lower_map: dict,
        level_out: dict = None,
    ):
        matched = bone_id_map.get(pmx_idx)
        level = 1
        if matched is None:
            matched = name_j_map.get(pmx_name)
            level = 2
        if matched is None:
            matched = pose_bones.get(pmx_name)
            level = 3
        if matched is None:
            matched = lower_map.get(pmx_name.lower())
            level = 4
        if matched is None:

            for bl_name, pb in lower_map.items():
                base = _BASE_NAME_RE.sub("", bl_name)
                if base == pmx_name.lower():
                    matched = pb
                    level = 5
                    break
        if level_out is not None:
            level_out[pmx_idx] = level if matched else 6
        return matched

    def get_match_report(self) -> str:
        total = self.bone_count
        matched = sum(1 for lv in self._last_match_level.values() if lv < 6)
        counts = {i: 0 for i in range(1, 7)}
        for lv in self._last_match_level.values():
            counts[lv] = counts.get(lv, 0) + 1
        return (
            f"Bone mapping: {matched}/{total} matched "
            f"(L1:{counts[1]} L2:{counts[2]} L3:{counts[3]} "
            f"L4:{counts[4]} L5:{counts[5]} L6:{counts[6]} unmapped)"
        )

    def init_mapping(self, lib, ctx, armature, bone_count: int) -> bool:
        self.bone_count = bone_count
        names = self.get_bone_names(lib, ctx)
        pose_bones = armature.pose.bones

        name_j_map = {}
        bone_id_map = {}
        lower_map = {}
        for pb in pose_bones:
            mmd = getattr(pb, "mmd_bone", None)
            if mmd:
                n = getattr(mmd, "name_j", "")
                if n:
                    name_j_map[n] = pb
                bid = getattr(mmd, "bone_id", -1)
                if bid >= 0:
                    bone_id_map[bid] = pb
            lower_map[pb.name.lower()] = pb

        mat3_array = (ctypes.c_float * (bone_count * 9))()
        out_mat = (ctypes.c_float * 9)()
        _unused_offset = (ctypes.c_float * 3)()

        self._last_match_level.clear()
        for pmx_idx, pmx_name in enumerate(names):
            matched_pose = self.match_bone(
                pmx_idx,
                pmx_name,
                pose_bones,
                name_j_map,
                bone_id_map,
                lower_map,
                self._last_match_level,
            )
            lv = self._last_match_level.get(pmx_idx, 6)
            if lv <= 4:
                print(f"[MMBP] bone[{pmx_idx}] '{pmx_name}' matched at L{lv}")
            elif lv == 5:
                print(
                    f"[MMBP] bone[{pmx_idx}] '{pmx_name}' matched at L5 (regex fallback)"
                )
            else:
                print(f"[MMBP] bone[{pmx_idx}] '{pmx_name}' NOT MATCHED (L6)")
            if matched_pose:
                self.bone_map[pmx_idx] = matched_pose
                matched_pose.rotation_mode = "QUATERNION"
                bl_bone = matched_pose.bone
            else:
                bl_bone = None

            if bl_bone:
                ml = bl_bone.matrix_local
                bone_local = (ctypes.c_float * 16)(
                    ml[0][0],
                    ml[0][1],
                    ml[0][2],
                    ml[0][3],
                    ml[1][0],
                    ml[1][1],
                    ml[1][2],
                    ml[1][3],
                    ml[2][0],
                    ml[2][1],
                    ml[2][2],
                    ml[2][3],
                    ml[3][0],
                    ml[3][1],
                    ml[3][2],
                    ml[3][3],
                )
                lib.mmbp_compute_rest_info(bone_local, out_mat, _unused_offset)
                base = pmx_idx * 9
                for i in range(9):
                    mat3_array[base + i] = out_mat[i]
            else:
                mat3_array[pmx_idx * 9] = 1
                mat3_array[pmx_idx * 9 + 4] = 1
                mat3_array[pmx_idx * 9 + 8] = 1

        lib.mmbp_set_bone_rest_info(ctx, mat3_array, bone_count)
        return True

    def write_to_blender(self, locs, quats, lib=None, ctx=None):

        try:
            loc_view = memoryview(locs).cast("B")
            quat_view = memoryview(quats).cast("B")
        except Exception:
            loc_view = quat_view = None
        locs_changed = self._last_locs is None or loc_view != self._last_locs
        quats_changed = self._last_quats is None or quat_view != self._last_quats
        if not locs_changed and not quats_changed:
            return
        if loc_view is not None:
            self._last_locs = memoryview(bytes(loc_view)).cast("B")
            self._last_quats = memoryview(bytes(quat_view)).cast("B")

        stale_keys = []
        for pmx_idx, bone in self.bone_map.items():
            try:
                i3 = pmx_idx * 3
                i4 = pmx_idx * 4
                bone.location = (
                    locs[i3] if math.isfinite(locs[i3]) else 0.0,
                    locs[i3 + 1] if math.isfinite(locs[i3 + 1]) else 0.0,
                    locs[i3 + 2] if math.isfinite(locs[i3 + 2]) else 0.0,
                )
                bone.rotation_quaternion = (
                    quats[i4] if math.isfinite(quats[i4]) else 1.0,
                    quats[i4 + 1] if math.isfinite(quats[i4 + 1]) else 0.0,
                    quats[i4 + 2] if math.isfinite(quats[i4 + 2]) else 0.0,
                    quats[i4 + 3] if math.isfinite(quats[i4 + 3]) else 0.0,
                )
            except ReferenceError:

                stale_keys.append(pmx_idx)
        for key in stale_keys:
            del self.bone_map[key]

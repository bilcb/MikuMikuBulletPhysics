import bpy
import os
import ctypes
from ..core.state import get_manager, resolve_armature

_manager = get_manager()


_WARNING_BUF = 256


def _report_warnings(operator, st, context):

    if st is None or st.lib is None or st.ctx is None:
        return
    try:
        count = st.lib.mmbp_get_warning_count(st.ctx)
    except Exception:
        return
    if count <= 0:
        return
    buf = ctypes.create_string_buffer(_WARNING_BUF)
    visible = min(count, 10)
    for i in range(visible):
        try:
            st.lib.mmbp_get_warning(st.ctx, i, buf, _WARNING_BUF)
        except Exception:
            break
        msg = buf.value.decode("utf-8", errors="replace")
        operator.report({"WARNING"}, msg)
    if count > visible:
        operator.report(
            {"WARNING"}, f"({count - visible} more warnings omitted — see console)"
        )


class MMBP_OT_load_base(bpy.types.Operator):
    bl_options = {"REGISTER"}
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def _check_ext(self, path: str, ext: str) -> bool:
        return path.lower().endswith(ext)

    def _load(self, st, filepath, arm):
        raise NotImplementedError

    def execute(self, context):
        if not self._check_ext(self.filepath, self._expected_ext):
            self.report({"ERROR"}, self._ext_error_msg)
            return {"CANCELLED"}
        arm = resolve_armature(context)
        if arm is None:
            self.report({"ERROR"}, "Select an armature")
            return {"CANCELLED"}
        st = _manager.get_or_create(arm)
        if st.running:
            st.running = False
            st.restore_all()
        try:
            self._load(st, self.filepath, arm)
        except Exception as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}

        _report_warnings(self, st, context)
        self.report({"INFO"}, self._success_msg(st))
        if context.area:
            context.area.tag_redraw()
        return {"FINISHED"}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


class MMBP_OT_load_model(MMBP_OT_load_base):
    bl_idname = "mmbp.load_model"
    bl_label = "Load PMX Model"
    _expected_ext = ".pmx"
    _ext_error_msg = "Select a .pmx file"

    def _load(self, st, filepath, arm):
        st.load_model(filepath, arm)

    def _success_msg(self, st):
        return f"Loaded PMX: {st.bone_count} bones, {st.lib.mmbp_get_rigid_body_count(st.ctx)} rigid bodies"


class MMBP_OT_load_motion(MMBP_OT_load_base):
    bl_idname = "mmbp.load_motion"
    bl_label = "Load VMD Motion"
    _expected_ext = ".vmd"
    _ext_error_msg = "Select a .vmd file"

    def _load(self, st, filepath, arm):
        if st.bone_count == 0:
            raise RuntimeError("Load PMX first")
        if not st.ctx:
            raise RuntimeError("Physics context destroyed, reload PMX first")
        st.load_motion(filepath)

    def _success_msg(self, st):
        return f"VMD loaded: {os.path.basename(self.filepath)}"


class MMBP_OT_load_pose(MMBP_OT_load_base):
    bl_idname = "mmbp.load_pose"
    bl_label = "Load VPD Pose"
    _expected_ext = ".vpd"
    _ext_error_msg = "Select a .vpd file"

    def _load(self, st, filepath, arm):
        if st.bone_count == 0:
            raise RuntimeError("Load PMX first")
        if not st.ctx:
            raise RuntimeError("Physics context destroyed, reload PMX first")
        st.load_pose(filepath)

        try:
            pose_name = os.path.splitext(os.path.basename(filepath))[0]
            if arm and arm.pose_library:

                existing = [
                    m for m in arm.pose_library.pose_markers if m.name == pose_name
                ]
                if not existing:
                    marker = arm.pose_library.pose_markers.new(pose_name)
                    marker.frame = bpy.context.scene.frame_current
        except (AttributeError, ReferenceError):
            pass

    def _success_msg(self, st):
        return f"VPD loaded: {os.path.basename(self.filepath)}"


CLASSES = [MMBP_OT_load_model, MMBP_OT_load_motion, MMBP_OT_load_pose]

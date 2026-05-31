import bpy
from ..core.state import get_manager, resolve_armature

_manager = get_manager()


class MMP_OT_load_model(bpy.types.Operator):
    bl_idname = "mmp.load_model"
    bl_label = "Load PMX Model"
    bl_options = {'REGISTER'}
    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    def execute(self, context):
        if not self.filepath.lower().endswith('.pmx'):
            self.report({'ERROR'}, "Select a .pmx file")
            return {'CANCELLED'}
        arm = resolve_armature(context)
        if arm is None:
            self.report({'ERROR'}, "Select an armature")
            return {'CANCELLED'}
        st = _manager.get_or_create(arm)
        try:
            st.load_model(self.filepath, arm)
        except RuntimeError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        self.report({'INFO'}, f"Loaded PMX: {st.bone_count} bones, {st.lib.mmp_get_rigid_body_count(st.ctx)} rigid bodies")
        if context.area:
            context.area.tag_redraw()
        return {'FINISHED'}
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class MMP_OT_load_motion(bpy.types.Operator):
    bl_idname = "mmp.load_motion"
    bl_label = "Load VMD Motion"
    bl_options = {'REGISTER'}
    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    def execute(self, context):
        if not self.filepath.lower().endswith('.vmd'):
            self.report({'ERROR'}, "Select a .vmd file")
            return {'CANCELLED'}
        arm = resolve_armature(context)
        if arm is None:
            self.report({'ERROR'}, "Select an armature")
            return {'CANCELLED'}
        st = _manager.get_or_create(arm)
        if st.bone_count == 0:
            self.report({'ERROR'}, "Load PMX first")
            return {'CANCELLED'}
        try:
            st.load_motion(self.filepath)
        except RuntimeError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        self.report({'INFO'}, "VMD loaded")
        if context.area:
            context.area.tag_redraw()
        return {'FINISHED'}
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


CLASSES = [MMP_OT_load_model, MMP_OT_load_motion]

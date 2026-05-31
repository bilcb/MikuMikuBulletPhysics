import bpy, time
from ..core.state import get_manager, resolve_armature
from ..core.loop import timer

_manager = get_manager()


class MMP_OT_start(bpy.types.Operator):
    bl_idname = "mmp.start"
    bl_label = "Start Physics"
    bl_options = {'REGISTER'}

    def execute(self, context):
        arm = resolve_armature(context)
        if arm is None:
            self.report({'ERROR'}, "Select an armature")
            return {'CANCELLED'}
        st = _manager.get_or_create(arm)
        if st.bone_count == 0:
            self.report({'ERROR'}, "Load PMX first")
            return {'CANCELLED'}
        if st.running:
            # Fix stale running flag from previous session
            st.running = False
            st.restore_all()
        st.disable_all()
        st.running = True
        st.last_time = time.perf_counter()
        st.accumulator = 0.0
        bpy.app.timers.register(timer, first_interval=0.0)
        self.report({'INFO'}, "Physics started")
        if context.area:
            context.area.tag_redraw()
        return {'FINISHED'}


class MMP_OT_stop(bpy.types.Operator):
    bl_idname = "mmp.stop"
    bl_label = "Stop Physics"
    bl_options = {'REGISTER'}

    def execute(self, context):
        for name in list(_manager.get_active_names()):
            st = _manager.get(name)
            if st and st.running:
                st.running = False
                st.restore_all()
        self.report({'INFO'}, "Physics stopped")
        if context.area:
            context.area.tag_redraw()
        return {'FINISHED'}


CLASSES = [MMP_OT_start, MMP_OT_stop]

import bpy
from ..core.state import get_manager

_manager = get_manager()


class MMP_PT_main(bpy.types.Panel):
    bl_label = "MMP Physics"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "MMP"

    def draw(self, context):
        layout = self.layout

        arm = context.active_object
        running = False
        st = None

        if arm and arm.type == 'ARMATURE':
            st = _manager.get(arm.name)
            if st:
                running = st.running

        if not running:
            for name in _manager.get_active_names():
                s = _manager.get(name)
                if s and s.running:
                    running = True
                    st = s
                    break

        row = layout.row()
        sub = row.row()
        sub.enabled = not running
        if running:
            sub.operator("mmp.start", text="Running\u2026", depress=True)
        else:
            sub.operator("mmp.start", text="Start Physics")
        row.operator("mmp.stop", text="Stop")

        layout.separator()
        layout.operator("mmp.load_model", text="Load PMX")
        layout.operator("mmp.load_motion", text="Load VMD")
        layout.separator()
        if st and st.bone_count > 0:
            layout.label(text=f"Bones: {st.bone_count}")
        if st and st.lib:
            try:
                rb = st.lib.mmp_get_rigid_body_count(st.ctx)
                jt = st.lib.mmp_get_joint_count(st.ctx)
                if rb > 0 or jt > 0:
                    layout.label(text=f"RigidBodies: {rb}  Joints: {jt}")
            except Exception:
                pass


CLASSES = [MMP_PT_main]

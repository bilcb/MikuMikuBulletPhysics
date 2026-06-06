import bpy
from ..core.state import (
    get_manager,
    DEFAULT_GRAVITY,
    DEFAULT_SOLVER_ITERATIONS,
    DEFAULT_FIXED_TIMESTEP,
)

_manager = get_manager()


class MMBP_PT_main(bpy.types.Panel):
    bl_label = "MMBP Physics"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MMBP"

    def draw(self, context):
        layout = self.layout

        arm = context.active_object
        st = None

        if arm and arm.type == "ARMATURE":
            st = _manager.get(arm.name)

        if st is None:
            for name in _manager.get_active_names():
                s = _manager.get(name)
                if s and s.running:
                    st = s
                    break

        running = st is not None and st.running

        row = layout.row()
        sub = row.row()
        sub.enabled = not running
        if running:
            sub.operator("mmbp.start", text="Running\u2026", depress=True)
        else:
            sub.operator("mmbp.start", text="Start Physics")
        row.operator("mmbp.stop", text="Stop")

        layout.separator()

        layout.operator("mmbp.load_model", text="Load PMX")
        layout.operator("mmbp.load_motion", text="Load VMD")
        layout.operator("mmbp.load_pose", text="Load VPD Pose")

        layout.separator()

        scene = context.scene

        box = layout.box()
        box.label(text="Physics Settings", icon="PHYSICS")
        box.enabled = not running

        col = box.column(align=True)
        col.prop(scene, "mmbp_gravity", text="Gravity")
        col.prop(scene, "mmbp_solver_iterations", text="Solver Iterations")

        col2 = box.column(align=True)
        col2.prop(scene, "mmbp_fixed_timestep", text="Physics Rate")
        col2.prop(scene, "mmbp_scale", text="Scale (0=auto)")

        col3 = box.column(align=True)
        col3.prop(scene, "mmbp_max_substeps", text="Max Substeps")

        col4 = box.column(align=True)
        col4.prop(scene, "mmbp_spring_damping", text="Spring Damping")

        layout.separator()

        box2 = layout.box()
        box2.label(text="Performance", icon="SORTTIME")
        col5 = box2.column(align=True)
        col5.prop(scene, "mmbp_write_interval", text="Write Bones Every N Steps")

        if st and st.bone_count > 0:
            layout.label(text=f"Bones: {st.bone_count}")
        if st and st.lib and st.ctx:
            try:
                rb = st.lib.mmbp_get_rigid_body_count(st.ctx)
                jt = st.lib.mmbp_get_joint_count(st.ctx)
                ik = st.lib.mmbp_get_ik_count(st.ctx)
                if rb > 0 or jt > 0:
                    layout.label(text=f"RigidBodies: {rb}  Joints: {jt}")
                if ik > 0:
                    layout.label(text=f"IK Bones: {ik}")
            except Exception:
                pass

        box2 = layout.box()
        box2.label(text="Debug", icon="PREFERENCES")
        row2 = box2.row(align=True)
        row2.prop(scene, "mmbp_log_level", text="Log Level")


CLASSES = [MMBP_PT_main]

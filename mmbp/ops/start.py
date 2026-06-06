import bpy, time
from ..core.state import (
    get_manager,
    resolve_armature,
    get_physics_timestep,
    get_physics_gravity,
    DEFAULT_GRAVITY,
    DEFAULT_SOLVER_ITERATIONS,
    DEFAULT_FIXED_TIMESTEP,
)
from ..core.loop import timer, register_frame_handler

_manager = get_manager()


class MMBP_OT_start(bpy.types.Operator):
    bl_idname = "mmbp.start"
    bl_label = "Start Physics"
    bl_options = {"REGISTER"}

    def execute(self, context):
        arm = resolve_armature(context)
        if arm is None:
            self.report({"ERROR"}, "Select an armature")
            return {"CANCELLED"}
        st = _manager.get_or_create(arm)
        if st.bone_count == 0:
            self.report({"ERROR"}, "Load PMX first")
            return {"CANCELLED"}
        if st.running:

            self.report({"INFO"}, "Physics is already running (no change)")
            return {"FINISHED"}

        scene = context.scene
        gravity = get_physics_gravity(scene)
        solver_iters = getattr(
            scene, "mmbp_solver_iterations", DEFAULT_SOLVER_ITERATIONS
        )
        fixed_step = get_physics_timestep(scene)
        max_substeps = getattr(scene, "mmbp_max_substeps", 10)
        spring_damping = getattr(scene, "mmbp_spring_damping", 0.3)
        try:
            log_level = int(getattr(scene, "mmbp_log_level", "2"))
        except (ValueError, TypeError):
            log_level = 2

        try:
            st.lib.mmbp_set_gravity(st.ctx, *gravity)
            st.lib.mmbp_set_solver_iters(st.ctx, solver_iters)
            st.lib.mmbp_set_fixed_timestep(st.ctx, fixed_step)
            st.lib.mmbp_set_max_substeps(st.ctx, max_substeps)
            st.lib.mmbp_set_spring_damping(st.ctx, spring_damping)
            st.lib.mmbp_set_log_level(log_level)
        except Exception as e:
            self.report({"ERROR"}, f"Failed to apply physics config: {e}")
            return {"CANCELLED"}

        st.disable_all()
        st.running = True
        st.last_time = time.perf_counter()
        st.accumulator = 0.0
        register_frame_handler()
        if not bpy.app.timers.is_registered(timer):
            bpy.app.timers.register(timer, first_interval=0.0)
        self.report({"INFO"}, "Physics started")
        if context.area:
            context.area.tag_redraw()
        return {"FINISHED"}


class MMBP_OT_stop(bpy.types.Operator):
    bl_idname = "mmbp.stop"
    bl_label = "Stop Physics"
    bl_options = {"REGISTER"}

    def execute(self, context):
        for name in list(_manager.get_active_names()):
            st = _manager.get(name)
            if st and st.running:
                st.running = False

                try:
                    st.reset_to_animation(context.scene.frame_current)
                except Exception as e:
                    print(f"[MMBP] reset_to_animation failed during stop: {e}")
                st.restore_all()
        self.report({"INFO"}, "Physics stopped — bones restored to animation pose")
        if context.area:
            context.area.tag_redraw()
        return {"FINISHED"}


CLASSES = [MMBP_OT_start, MMBP_OT_stop]

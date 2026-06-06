import time, bpy
from .state import (
    get_manager,
    get_physics_timestep,
    DEFAULT_FIXED_TIMESTEP,
)

_manager = get_manager()
_last_ui_update = 0.0
_UI_UPDATE_INTERVAL = 1.0 / 30.0
_JUMP_THRESHOLD = 1.0


MAX_FRAME_SKIP_SECONDS = 0.25


_frame_history = {}


def timer():

    active = _manager.has_running()
    if not active:
        return None
    _tick_impl()

    interval = DEFAULT_FIXED_TIMESTEP
    for name in _manager.get_active_names():
        st = _manager.get(name)
        if st and st.running:
            try:
                dt = st.lib.mmbp_get_fixed_timestep(st.ctx)
                if 0 < dt < interval:
                    interval = dt
            except Exception:
                pass
    return interval


def _tick_impl():
    for name in list(_manager.get_active_names()):
        st = _manager.get(name)
        if st is None or not st.running:
            continue
        try:
            now = time.perf_counter()
            if st.last_time > 0:

                st.accumulator += min(now - st.last_time, MAX_FRAME_SKIP_SECONDS)
            st.last_time = now
            scene = st._find_scene()
            fixed_step = get_physics_timestep(scene)
            frame = float(scene.frame_current)
            stepped = False
            write_interval = max(1, getattr(scene, "mmbp_write_interval", 1))

            if not hasattr(st, "_write_counter"):
                st._write_counter = 0
            while st.accumulator >= fixed_step:
                ok = st.lib.mmbp_step(
                    st.ctx, frame, fixed_step, st.locs, st.quats, st.bone_count
                )
                st.accumulator -= fixed_step
                st._write_counter += 1
                if not ok:
                    err = st.lib.mmbp_get_last_error(st.ctx)
                    if err:
                        msg = err.decode("utf-8", errors="replace")
                        print(f"[MMBP] mmbp_step failed: {msg}")
                    st.running = False
                    st.restore_all()
                    return
                stepped = True
            if stepped and st.armature:
                if st._write_counter >= write_interval:
                    try:
                        st.write_to_blender()
                        st._write_counter = 0
                    except ReferenceError:

                        st.running = False
                        try:
                            st.restore_all()
                        except Exception:
                            pass
                        return
        except Exception as e:
            print(f"[MMBP] tick exception: {e}")
            import traceback

            traceback.print_exc()
            st.running = False
            try:
                st.restore_all()
            except Exception as e:
                print(f"[MMBP] restore_all failed during cleanup: {e}")


@bpy.app.handlers.persistent
def _on_frame_change_post(scene):
    for name in _manager.get_active_names():
        st = _manager.get(name)
        if st is None or not st.running:
            continue
        try:
            ptr = st.armature.as_pointer() if st.armature else None
        except ReferenceError:
            continue
        if ptr is None:
            continue
        frame = float(scene.frame_current)
        last = _frame_history.get(ptr, frame)
        _frame_history[ptr] = frame
        if abs(frame - last) > _JUMP_THRESHOLD:
            try:
                st.reset_to_animation(frame)
                st.accumulator = 0.0
            except Exception as e:
                print(f"[MMBP] frame_change reset failed: {e}")


def register_frame_handler():
    if _on_frame_change_post not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_on_frame_change_post)


def unregister_frame_handler():
    if _on_frame_change_post in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_on_frame_change_post)
    _frame_history.clear()

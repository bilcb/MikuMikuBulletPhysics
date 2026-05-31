import time, bpy
from .state import get_manager

_manager = get_manager()

_ui_frame_counter = 0
_UI_UPDATE_INTERVAL = 4  # every 4 frames ≈ 30Hz at 120Hz physics

def timer():
    active = any(
        (_manager.get(n) is not None and _manager.get(n).running)
        for n in _manager.get_active_names()
    )
    if not active:
        return None
    _tick_impl()
    return 1.0 / 120.0 * 0.5


def _tick_impl():
    for name in list(_manager.get_active_names()):
        st = _manager.get(name)
        if st is None or not st.running:
            continue
        try:
            now = time.perf_counter()
            if st.last_time > 0:
                st.accumulator += min(now - st.last_time, 0.25)
            st.last_time = now
            fixed_step = 1.0 / 120.0
            frame = float(bpy.context.scene.frame_current)
            dll_count = st.lib.mmp_get_bone_count(st.ctx)
            if dll_count != st.bone_count:
                print(f"[MMP] bone mismatch: dll={dll_count} st={st.bone_count}")
                st.running = False
                st.restore_all()
                continue
            stepped = False
            while st.accumulator >= fixed_step:
                ok = st.lib.mmp_step(st.ctx, frame, fixed_step,
                                      st.locs, st.quats, st.bone_count)
                st.accumulator -= fixed_step
                if not ok:
                    err = st.lib.mmp_get_last_error(st.ctx)
                    if err:
                        msg = err.decode('utf-8', errors='replace')
                        print(f"[MMP] mmp_step failed: {msg}")
                    st.running = False
                    st.restore_all()
                    return
                stepped = True
            if stepped and st.armature:
                st.write_to_blender()
                global _ui_frame_counter
                _ui_frame_counter += 1
                if _ui_frame_counter >= _UI_UPDATE_INTERVAL:
                    bpy.context.view_layer.update()
                    _ui_frame_counter = 0
        except Exception as e:
            print(f"[MMP] tick exception: {e}")
            import traceback
            traceback.print_exc()
            st.running = False
            try:
                st.restore_all()
            except Exception as e:
                print(f"[MMP] restore_all failed during cleanup: {e}")

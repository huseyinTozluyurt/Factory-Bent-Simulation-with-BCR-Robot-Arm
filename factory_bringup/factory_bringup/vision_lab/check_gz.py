"""
Check that the vision lab can drive the running pipeline lab world in Gazebo:
    ./run_vision_lab.sh check          (with ./run_vision_lab.sh sim --world pipeline running)

It tests, one by one: the gz-transport Python bindings, the world's services, spawning a product
(Python bindings, then the gz command line), moving it with set_pose_vector, one station camera image,
and removing the product again. Each line says OK or what failed.
"""
import sys
import time

from factory_bringup.vision_lab import pipeline_world as P


def main() -> int:
    ok_all = True

    def line(ok, text):
        nonlocal ok_all
        ok_all &= bool(ok)
        print(f"  [{'OK  ' if ok else 'FAIL'}] {text}", flush=True)

    print("Vision lab <-> Gazebo check (pipeline world)")
    try:
        from factory_bringup.vision_lab.pipeline_sim import GzBackend, BELT_TOP
        gz = GzBackend()
        import gz.transport13 as gt
        line(True, f"Python bindings: {gt.__file__}")
    except ImportError as exc:
        line(False, str(exc))
        return 1
    if not gz.running():
        line(False, f"no /world/{P.WORLD_NAME}/... services: start ./run_vision_lab.sh sim --world pipeline first")
        return 1
    line(True, f"world {P.WORLD_NAME} is running")

    from factory_bringup.product_catalog import product_sdf
    name = f"check_cereal_box_{int(time.time())}"
    x, y, _ = P.path_xy("S1", 2.0)
    q = (0.0, 0.0, 0.0, 1.0)
    f = gz.m["EntityFactory"]()
    f.sdf, f.name, f.allow_renaming = product_sdf(name, "cereal_box"), name, False
    gz._pose(f.pose, x, y, BELT_TOP + 0.2, q)
    t = time.time()
    py_ok = gz._req("create", f, "Boolean", 10000)
    line(py_ok, f"spawn through the Python bindings ({time.time() - t:.1f} s)")
    created = py_ok
    if not py_ok:
        t = time.time()
        cli_ok = gz._cli_req("create", f)
        line(cli_ok, f"spawn through `gz service` ({time.time() - t:.1f} s)" + ("" if cli_ok else f": {gz.last_error}"))
        created = cli_ok
    if created:
        v = gz.m["Pose_V"]()
        p = v.pose.add()
        p.name = name
        gz._pose(p, x + 1.0, y, BELT_TOP + 0.2, q)
        line(gz._req("set_pose_vector", v, "Boolean", 3000), "move it with set_pose_vector (Python bindings)")
        t, n = time.time(), 0
        while time.time() - t < 3.0:                 # how often can the lab move things?
            p.position.x += 0.01
            n += gz._req("set_pose_vector", v, "Boolean", 2000)
        rate = n / (time.time() - t)
        line(rate >= 5, f"pose updates: {rate:.0f} per second (the belts look smooth from ~10 per second)")

    got = []
    try:
        gz.subscribe_camera(P.camera_topic("A1"), lambda img, t: got.append(img.shape))
        end = time.time() + 10
        while not got and time.time() < end:
            time.sleep(0.2)
        line(bool(got), f"camera {P.camera_topic('A1')}: " + (f"image {got[0]}" if got else "no image in 10 s"))
    except Exception as exc:
        line(False, f"camera {P.camera_topic('A1')}: {exc}")

    if created:
        e = gz.m["Entity"]()
        e.name, e.type = name, gz.m["Entity"].MODEL
        line(gz._req("remove", e, "Boolean", 5000) or gz._cli_req("remove", e), "remove the test product")
    print("All OK: the labs can run in Gazebo." if ok_all else
          "Something failed: send this output. Meanwhile the labs run without Gazebo with --fake.")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())

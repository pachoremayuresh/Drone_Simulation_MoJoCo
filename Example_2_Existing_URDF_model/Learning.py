import time

import mujoco
import numpy as np
import mujoco.viewer


def set_initial_camera_view(viewer, model, data):
    # Set the camera to focus on a specific position in 3D space
        viewer.cam.lookat[:] = [0.0, 0.0, 1.0]  # Example position in 3D space (x, y, z)
        viewer.cam.distance = 5.0  # Set an appropriate distance
        viewer.cam.elevation = -20  # Set elevation angle
        viewer.cam.azimuth = 90  # Set azimuth angle
        pass


m = mujoco.MjModel.from_xml_path("Example_2_Existing_URDF_model/fiberthex/scene.xml")
d = mujoco.MjData(m)


with mujoco.viewer.launch_passive(m, d) as viewer:
    set_initial_camera_view(viewer, m, d)
    start = time.time()
    while viewer.is_running() and time.time() - start < 0.05:
        step_start = time.time()
        mujoco.mj_step(m, d)
        print(d.sensordata)
        viewer.sync()
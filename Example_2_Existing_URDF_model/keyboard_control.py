
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

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

class PDController:
    def __init__(self, kp, kd, setpoint):
        self.kp = kp
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        pass

    def compute(self, current_value):
        error = self.setpoint - current_value
        derivative = error - self.prev_error
        output = self.kp * error + self.kd * derivative
        self.prev_error = error
        return output
    
    pass
            

m = mujoco.MjModel.from_xml_path("Example_2_Existing_URDF_model/fiberthex/scene.xml")
d = mujoco.MjData(m)

d.ctrl[:6] = 4 # Set the thruster values to 0.5
size = d.ctrl.shape

kp = 0.5 # Proportional gain
kd = 0.1 # Derivative gain
setpoint = 0 # Vertical Speed should be 0

pd_controller = PDController(kp, kd, setpoint)

with mujoco.viewer.launch_passive(m, d) as viewer:
    set_initial_camera_view(viewer, m, d)
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        ###Control loop to stabilize drone height.
        measured_speed = d.qvel[2]
        #print(measured_speed)
        control_signal = pd_controller.compute(measured_speed)
        #print(control_signal)
        d.ctrl[:6] = d.ctrl[:6] + control_signal
        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
            pass


        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            pass
        pass
    pass


    

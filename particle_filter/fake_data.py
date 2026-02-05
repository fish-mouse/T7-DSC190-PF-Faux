import math
import random
import numpy as np

class FakeWorld:
    def __init__(self, landmarks=None, motion_noise_std=(0.02,0.02,0.01), meas_noise_std=(0.1, 0.05)):
        self.landmarks = landmarks or [(2.0, 1.0), (0.0, -2.0), (4.0, -1.5), (-3.0, 2.0)]
        self.motion_sigma = motion_noise_std
        self.range_sigma, self.bearing_sigma = meas_noise_std

    def generate_trajectory(self, steps, control_fn, start_pose=(0.0,0.0,0.0)):
        pose = list(start_pose)
        history = []
        for i in range(steps):
            ctrl = control_fn(i)
            if len(ctrl) == 2:
                v, omega = ctrl
                dt = 1.0
                dx = v * math.cos(pose[2]) * dt
                dy = v * math.sin(pose[2]) * dt
                dtheta = omega * dt
            else:
                dx, dy, dtheta = ctrl

            dx_n = dx + random.gauss(0, self.motion_sigma[0])
            dy_n = dy + random.gauss(0, self.motion_sigma[1])
            dtheta_n = dtheta + random.gauss(0, self.motion_sigma[2])

            pose[0] += dx_n
            pose[1] += dy_n
            pose[2] += dtheta_n

            odom = (dx_n, dy_n, dtheta_n)
            meas = self._sense(pose)
            history.append((tuple(pose), odom, meas))
        return history

    def _sense(self, pose):
        x, y, th = pose
        measurements = []
        for lx, ly in self.landmarks:
            dx = lx - x
            dy = ly - y
            r = math.hypot(dx, dy) + random.gauss(0, self.range_sigma)
            bearing = math.atan2(dy, dx) - th + random.gauss(0, self.bearing_sigma)
            bearing = (bearing + math.pi) % (2*math.pi) - math.pi
            measurements.append((r, bearing, (lx, ly)))
        return measurements
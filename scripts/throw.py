from ur5py.ur5 import UR5Robot
import numpy as np
import time
import pdb

from roboticstoolbox.tools.trajectory import quintic
from roboticstoolbox.tools.trajectory import trapezoidal

robot = UR5Robot(ip="192.168.131.69", gripper=2)
current_pose = np.array(robot.get_pose(convert=False))

end_pose = current_pose.copy()
end_pose[1] -= 0.4


# def helper_sine(ini_start, ini_end, num):
#     time = np.linspace(0, 1, num)
#     speed = np.sin(np.pi * time)
#     position = np.cumsum(speed)
#     scaled_position = position - position[0]
#     scaled_position = scaled_position / scaled_position[-1]
#     scaled_position = ini_start + (ini_end - ini_start) * scaled_position
#     # full_cycle_pos = np.concatenate((scaled_position, scaled_position[::-1]), axis=0)
#     full_cycle_pos = scaled_position
#     return full_cycle_pos


def helper_quintic(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    for i in range(6):
        trajectory = quintic(ini_start[i], ini_end[i], time)
        positions = trajectory.q
        full_cycle_pos[:, i] = positions
    return full_cycle_pos


def helper_trapezoidal(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    for i in range(6):
        trajectory = trapezoidal(ini_start[i], ini_end[i], time)
        positions = trajectory.q
        full_cycle_pos[:, i] = positions
    return full_cycle_pos


##### Helper end
poses = helper_quintic(current_pose, end_pose, 550)
pdb.set_trace()
for i, row in enumerate(poses):
    print(i)
    start_time = time.time()
    robot.servo_pose(row, time=0.02, convert=False)
    if i == 100:
        robot.gripper.open()
    while time.time() - start_time < 0.002:
        pass
robot.stop_joint(3)

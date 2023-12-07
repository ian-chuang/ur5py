import numpy as np
from ur5py.ur5 import UR5Robot
import time
import pdb

robot = UR5Robot(ip="192.168.131.69", gripper=2)
current_pose = np.array(robot.get_pose(convert=False))
x = np.array([0.8, 0.2, 0.3])
y = np.array([1, 1, 1])
current_pose[3:] = [0, 0, 0]
robot.servo_pose(current_pose)

from ur5py.ur5 import UR5Robot
import numpy as np
import time
import math
import pdb
from tqdm import tqdm
import cowsay
import os
import sympy
import time
import datetime
from random import choice, uniform
from roboticstoolbox.tools.trajectory import quintic
from roboticstoolbox.tools.trajectory import trapezoidal

final_array = np.loadtxt()
runtime = final_array[-1][0]

robot = UR5Robot(ip="192.168.131.69", gripper=2)

for i, arr in enumerate(tqdm(final_array)):
    if i == 0:
        continue
    time_delta = 0.002
    robot.servo_joint(arr, time=time_delta, gain=2000)
    current_time = time.time()
    while time.time() - current_time < time_delta:
        pass
robot.stop_joint()

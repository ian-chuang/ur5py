from ur5py.ur5 import UR5Robot
import numpy as np
import time
import pdb

robot = UR5Robot(ip="192.168.131.69", gripper=2)
current_pose = np.array(robot.get_pose(convert=False))
end_pose = current_pose
end_pose[2] -= 0.01
poses = np.linspace(current_pose, end_pose, 20000)
pdb.set_trace()
for row in poses:
    pdb.set_trace()
    start_time = time.time()
    robot.servo_pose(row, time=0.02, convert=False)
    time.sleep(0.02)

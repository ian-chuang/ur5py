from ur5py.ur5 import UR5Robot
import numpy as np
import time
import pdb

robot = UR5Robot(ip="192.168.131.69", gripper=2)
current_pose = np.array(robot.get_pose(convert=False))

end_pose = current_pose.copy()
end_pose[1] += 0.2
poses = np.linspace(current_pose, end_pose, 1000)
pdb.set_trace()
for i, row in enumerate(poses):
    start_time = time.time()
    robot.servo_pose(row, time=0.004, convert=False)
    if time.time - start_time < 0.004:
        pass

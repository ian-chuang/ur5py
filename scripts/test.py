from ur5py import UR5Robot
import numpy as np
import time

if __name__ == "__main__":
    ur = UR5Robot(gripper=2)
    # Testing nonblocking
    joints_original = ur.get_pose()
    joints_goal = joints_original.copy() 
    joints_goal[0] -= 0.2
    
    # Start moving to goal
    runtime = 5 # seconds
    joint_targets = np.linspace(joints_original, joints_goal, int(5/0.002), True)
    start_time = time.time()
    for j in joint_targets:
        ur.servo_pose(j, convert=False, gain=400)
        time.sleep(0.002)
        if time.time()-start_time > 2:
            break
    # Give new goal
    joint_targets = np.linspace(ur.get_joints(), joints_original, int(2/0.002), True)
    start_time = time.time()
    for j in joint_targets:
        ur.servo_pose(j, convert=False, gain=400)
        time.sleep(0.002)
    
    
    

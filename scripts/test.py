from ur5py import UR5Robot
import numpy as np
import time

if __name__ == "__main__":
    ur = UR5Robot(gripper=2)
    # Testing nonblocking
    joints = ur.get_joints()
    joints_goal = joints.copy() 
    joints_goal[0] -= 0.2
    
    # Start moving to goal
    ur.move_joint(joints_goal, vel=0.2, asyn=True)
    time.sleep(0.4)
    # Give new goal
    ur.stop_joint(3)
    ur.move_joint(joints)
    
    
    

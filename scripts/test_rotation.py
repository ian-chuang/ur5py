import numpy as np

# from ur5py.ur5 import UR5Robot
import time
import pdb
import roboticstoolbox
from autolab_core import RigidTransform


def time_from_v(initial, target, target_v, time_per_step):
    min = target_dist / target_v
    max = min * 2
    target_t = -1
    for t in np.arange(min, max, step=0.1):
        f = roboticstoolbox.tools.trajectory.quintic_func(0, target_dist, t)
        _, max_v, _ = f(t / 2)
        if abs(max_v - target_v) <= 0.05:
            target_t = t
            break
    return t / time_per_step


def do_throw(robot, end_pose, instant_vel):
    current_pose = robot.get_pose(convert=True)
    # current_pose[3:] = end_pose[3:]
    # current_pose = np.array([1, 1, 1, 0.32, 0.23, 0.324])
    # end_pose = np.array([3, 3, 2, 0.21, 0.234, 0.98])
    # instant_vel = np.array([0.1, 0.3, 0.5])
    intermediate_pose = np.zeros(6)
    P = current_pose[:3]
    A = end_pose[3:]
    B = end_pose[3:] + instant_vel
    AP = P - A
    AB = B - A
    intermediate_pose[:3] = A + np.dot(AP, AB) / np.dot(AB, AB) * AB
    intermediate_pose[3:] = end_pose[3:]
    robot.move_pose(intermediate_pose, convert=False)
    robot.move_pose(end_pose)
    poses = ...
    release = int(len(poses) // 2)
    for i, p in poses:
        curr_time = time.time()
        robot.servo_pose(p, time=0.002, convert=False)
        if i == release:
            robot.gripper.open()
        while time.time() - curr_time < 0.002:
            pass
        if i > release + 300:
            break
    robot.stop_joint(5)


current_pose = np.array([1, 1, 1, 0.32, 0.23, 0.324])
end_pose = np.array([3, 3, 2, 0.21, 0.234, 0.98])
instant_vel = np.array([0.1, 0.3, 0.5])
intermediate_pose = np.zeros(6)
P = current_pose[:3]
A = end_pose[3:]
B = end_pose[3:] + instant_vel
AP = P - A
AB = B - A
intermediate_pose[:3] = A + np.dot(AP, AB) / np.dot(AB, AB) * AB
intermediate_pose[3:] = end_pose[3:]

print(intermediate_pose)

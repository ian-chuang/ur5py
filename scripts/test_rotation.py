import numpy as np

from ur5py.ur5 import UR5Robot
import time
import pdb
import roboticstoolbox
from autolab_core import RigidTransform
from roboticstoolbox.tools.trajectory import (
    quintic_func,
    quintic,
    trapezoidal,
    trapezoidal_func,
)


def max_v(ini_pos, target_pos, t):
    f = trapezoidal_func(ini_pos, target_pos, t)
    _, max_v, _ = f(t / 2)
    return max_v


def time_from_v(ini_pos, target_pos, target_v, time_per_step):
    virtual_target = (target_pos) + (target_pos - ini_pos)
    dist = np.linalg.norm(virtual_target - ini_pos)
    target_v_norm = np.linalg.norm(target_v)
    min = dist / target_v_norm
    max = min * 2
    target_t = -1
    for t in np.arange(min, max, step=0.001):
        velocity = np.zeros(3)
        for i in range(3):
            velocity[i] = max_v(ini_pos[i], virtual_target[i], t)
        if np.linalg.norm(velocity - target_v) <= 0.05:
            target_t = t
            break
    num_of_points = int(target_t / time_per_step)

    final_traj = np.zeros((num_of_points, 3))
    for i in range(3):
        trajectory = trapezoidal(ini_pos[i], virtual_target[i], num_of_points)
        print(num_of_points)
        positions = trajectory.q
        print(positions)
        final_traj[:, i] = positions
    return final_traj


def do_throw(robot, end_pose, instant_vel):
    current_pose = robot.get_pose(convert=False)
    instant_vel = np.array(instant_vel)
    # current_pose[3:] = end_pose[3:]
    # current_pose = np.array([1, 1, 1, 0.32, 0.23, 0.324])
    # end_pose = np.array([3, 3, 2, 0.21, 0.234, 0.98])
    # instant_vel = np.array([0.1, 0.3, 0.5])
    intermediate_pose = np.zeros(6)
    P = np.array(current_pose[:3])
    A = np.array(end_pose[3:])
    B = end_pose[3:] + instant_vel
    AP = P - A
    AB = B - A
    intermediate_pose[:3] = A + np.dot(AP, AB) / np.dot(AB, AB) * AB
    intermediate_pose[3:] = end_pose[3:]
    poses = time_from_v(intermediate_pose, end_pose, instant_vel, 0.002)
    release = int(len(poses) // 2)
    poses = np.hstack(
        [poses, np.repeat(intermediate_pose[3:].reshape(1, -1), len(poses), 0)]
    )
    robot.move_pose(intermediate_pose, convert=False)
    for i, p in enumerate(poses):
        curr_time = time.time()
        robot.servo_pose(p, time=0.002, convert=False)
        if i == release:
            robot.gripper.open()
        while time.time() - curr_time < 0.002:
            pass
        if i > release + 300:
            break
    robot.stop_joint(5)


robot = UR5Robot(ip="192.168.131.69", gripper=2)
do_throw(robot, [0.8, 0.8, 0, np.pi, 0, 0], [0.05, 0, 0])

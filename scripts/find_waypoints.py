import numpy as np
from ur5py.ur5 import UR5Robot
import time
import pdb
from roboticstoolbox.tools.trajectory import quintic
from roboticstoolbox.tools.trajectory import trapezoidal

from autolab_core import RigidTransform
import numpy as np

from ur5py.ur5 import UR5Robot, RT2UR, UR2RT
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


def do_throw(robot: UR5Robot, end_pose, instant_vel):
    robot.move_pose(end_pose)
    instant_vel = np.array(instant_vel)
    intermediate_pose = np.array(RT2UR(end_pose))
    intermediate_pose[:3] -= instant_vel / np.linalg.norm(instant_vel) * 0.3
    robot.move_pose(intermediate_pose, convert=False)
    poses = time_from_v(
        intermediate_pose, np.array(RT2UR(end_pose)), instant_vel, 0.002
    )
    poses = np.hstack(
        [poses, np.repeat(intermediate_pose[3:].reshape(1, -1), len(poses), 0)]
    )
    release = int(len(poses) // 2)
    angles = []
    for i, p in enumerate(poses):
        curr_time = time.time()
        robot.servo_pose(p, time=0.002, convert=False)
        angles.append(robot.get_joints())
        if i == release:
            robot.gripper.open()
        while time.time() - curr_time < 0.002:
            pass
        # if i > release + 300:
        #     break
    np.savetxt("move.txt", angles)
    robot.stop_joint(5)


def find_waypoints(current_pose, start, goal, total_t):
    L = 0.8  # radius of dexterous sphere
    g = 9.8
    num_waypoints = 500

    poses = []
    # orientation = robot.get_pose(convert=False)[3:]
    orientation = current_pose[3:]

    def find_release_orientation(launch_points):
        # find the launch vector from a series of input locations
        release_p = launch_points[0]
        launch_vector = launch_points[1] - release_p
        D = (
            0
            - launch_vector[0] * release_p[0]
            - launch_vector[1] * release_p[1]
            - launch_vector[2] * release_p[2]
        )
        arb_point = np.array(
            [0, 0, -D / launch_vector[2]]
        )  # an arbitary point on the plane
        v1_on_plane = release_p - arb_point
        v2_on_plane = np.cross(launch_vector, v1_on_plane)

        # normalize
        launch_vector = launch_vector / np.linalg.norm(launch_vector)
        v1_on_plane = v1_on_plane / np.linalg.norm(v1_on_plane)
        v2_on_plane = v2_on_plane / np.linalg.norm(v2_on_plane)

        # return 3x3 orientation matrix of the tool's pose
        return np.vstack([launch_vector, v1_on_plane, v2_on_plane]).T

    # robot.get_pose().matrix # 4x4 matrix

    def find_pos(t, total_t, start, goal, g):
        # indicates the trajectory of the object once it is released with velocity and start position
        # assume velocity is the magnitude of launch
        v_x = (goal[0] - start[0]) / total_t
        v_y = (goal[1] - start[1]) / total_t
        v_z = ((goal[2] - start[2]) + 0.5 * g * total_t**2) / total_t

        x_t = start[0] + v_x * t
        y_t = start[1] + v_y * t
        z_t = start[2] + v_z * t - 0.5 * g * t**2
        # z_t = np.tan(theta) * x_t - (g * (x_t ** 2))/ (2 * (v0**2) * (np.cos(theta))**2)

        return x_t, y_t, z_t, v_x, v_y, v_z

    for dt in np.linspace(0, total_t, num_waypoints):
        x, y, z, vi_x, vi_y, vi_z = find_pos(dt, total_t, start, goal, g)
        if np.sqrt(x**2 + y**2 + z**2) < L:
            waypoints = [x, y, z]
            # waypoints.extend(orientation)
            poses.append(waypoints)

    # create transformation matrix
    m = np.eye(4)
    # launch_points = np.array([[-0.6, 0.2, 0], [-0.7, 0.3, 0.2]])
    poses = np.array(poses)
    m[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) @ find_release_orientation(
        poses
    )
    m[:3, 3] = np.vstack(poses[0]).reshape(-1)

    velocity = np.array([vi_x, vi_y, vi_z])
    return poses, m, velocity


robot = UR5Robot(ip="192.168.131.69", gripper=2)
current_pose = np.array(robot.get_pose(convert=False))

end_pose = current_pose.copy()
end_pose[1] -= 0.4
start = np.array([-0.6, 0.2, 0])
goal = np.array([0, -1, 0])
total_time = 0.5
poses_temp, m, v_i = find_waypoints(current_pose, start, goal, total_time)
# transform m to end pose
tsfm = RigidTransform(m[:3, :3], m[:3, 3])
do_throw(robot, tsfm, v_i)


def helper_sine(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    speed = np.sin(np.pi * time)
    position = np.cumsum(speed)
    scaled_position = position - position[0]
    scaled_position = scaled_position / scaled_position[-1]
    scaled_position = ini_start + (ini_end - ini_start) * scaled_position
    # full_cycle_pos = np.concatenate((scaled_position, scaled_position[::-1]), axis=0)
    full_cycle_pos = scaled_position
    return full_cycle_pos


def helper_quintic(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    trajectory = quintic(ini_start, ini_end, time)
    # positions = trajectory.y[:, 0]
    positions = trajectory.q
    # full_cycle_pos = np.concatenate((positions, positions[::-1]), axis=0)
    full_cycle_pos = positions
    return full_cycle_pos


def helper_trapezoidal(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    trajectory = trapezoidal(ini_start, ini_end, time)
    # positions = trajectory.y[:, 0]
    positions = trajectory.q
    # full_cycle_pos = np.concatenate((positions, positions[::-1]), axis=0)
    full_cycle_pos = positions
    return full_cycle_pos


###### Helper end

import numpy as np
from ur5py.ur5 import UR5Robot
import time
import pdb
from roboticstoolbox.tools.trajectory import quintic
from roboticstoolbox.tools.trajectory import trapezoidal

def find_waypoints(current_pose, start, goal, total_t):
    L = 0.8 # radius of dexterous sphere
    g = 9.8
    num_waypoints = 500
    
    poses = []
    #orientation = robot.get_pose(convert=False)[3:]
    orientation = current_pose[3:]
    
    def find_pos(t, total_t, start, goal, g):
    # indicates the trajectory of the object once it is released with velocity and start position
    # assume velocity is the magnitude of launch
        v_x = (goal[0] - start[0]) / total_t
        v_y = (goal[1] - start[1]) / total_t
        v_z = ((goal[2] - start[2]) + 0.5 * g * total_t **2) / total_t
        
        x_t = start[0] + v_x * t
        y_t = start[1] + v_y * t
        z_t = start[2] + v_z * t - 0.5 * g * t**2
        # z_t = np.tan(theta) * x_t - (g * (x_t ** 2))/ (2 * (v0**2) * (np.cos(theta))**2)
        
        return x_t, y_t, z_t, v_x, v_y, v_z
    
    for dt in np.linspace(0, total_t, num_waypoints):
        x, y, z, vi_x, vi_y, vi_z = find_pos(dt, total_t, start, goal, g)
        if (np.sqrt(x**2 + y**2 + z**2) < L):
            waypoints = [x, y, z]
            waypoints.extend(orientation)
            poses.append(waypoints)
            
    return poses

robot = UR5Robot(ip="192.168.131.69", gripper=2)
current_pose = np.array(robot.get_pose(convert=False))

end_pose = current_pose.copy()
end_pose[1] -= 0.4
start = current_pose[:3]
goal = np.array([0, -3, 0])
total_time = 1
poses = find_waypoints(current_pose, start, goal, total_time)
#poses = np.linspace(current_pose, end_pose, 250)
pdb.set_trace()
for i, row in enumerate(poses):
    print(i)
    start_time = time.time()
    robot.servo_pose(row, time=0.04, convert=False)
    # if i == 100:
    #     robot.gripper.open()
    while time.time() - start_time < 0.04:
        pass
robot.stop_joint(3)

def helper_sine(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    speed = np.sin(np.pi * time)
    position = np.cumsum(speed)
    scaled_position = position - position[0]
    scaled_position = scaled_position / scaled_position[-1]
    scaled_position = ini_start + (ini_end - ini_start)*scaled_position
    #full_cycle_pos = np.concatenate((scaled_position, scaled_position[::-1]), axis=0)
    full_cycle_pos = scaled_position
    return full_cycle_pos

def helper_quintic(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    trajectory = quintic(ini_start, ini_end, time)
    #positions = trajectory.y[:, 0]
    positions = trajectory.q
    #full_cycle_pos = np.concatenate((positions, positions[::-1]), axis=0)
    full_cycle_pos = positions
    return full_cycle_pos

def helper_trapezoidal(ini_start, ini_end, num):
    time = np.linspace(0, 1, num)
    trajectory = trapezoidal(ini_start, ini_end, time)
    #positions = trajectory.y[:, 0]
    positions = trajectory.q
    #full_cycle_pos = np.concatenate((positions, positions[::-1]), axis=0)
    full_cycle_pos = positions
    return full_cycle_pos
###### Helper end
import numpy as np

def find_waypoints(current_pose, start, goal, total_t):
    L = 0.8 # radius of dexterous sphere
    g = 9.8
    num_waypoints = 2000
    
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

if __name__ == "__main__":
    start = np.array([-0.6, 0.2, 0]) # starting position
    basket = np.array([0, -3, 0])
    total_time = 1
    poses = find_waypoints(start, basket, total_time)
    print(poses)
        
    
    
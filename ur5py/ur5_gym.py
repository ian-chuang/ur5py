import gymnasium as gym
from ur5py import UR5Robot, UR2RT, RT2UR

class GymUr(gym.Env):
    # Pronoucned jimer
    
    def __init__(self) -> None:
        super().__init__()
        self.robot = UR5Robot(gripper=2)
from ur5py.ur5 import UR5Robot
import numpy as np

if __name__ == "__main__":
    ur = UR5Robot("172.22.22.2")
    input("enter to enter freedrive")
    ur.start_teach()
    input("enter to end freedrive")
    ur.stop_teach()

import rtde_control
import time
from threading import Thread
import numpy as np
import time
import rtde_control
import threading
import cowsay
import socket
import pdb


class SuccRobotiq(object):
    def __init__(self, host_ip, gripper_port=63352) -> None:
        self.host_ip = host_ip
        self.gripper_port = gripper_port
        self.s_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_gripper.connect((host_ip, gripper_port))
        self.nonce_byte = 0
        self.calibrate()

    def calibrate(self, advanced=True):
        self.s_gripper.sendall(b"SET ACT 0\n")
        self.s_gripper.recv(4)
        time.sleep(0.1)
        self.s_gripper.sendall(b"SET ACT 1\n")
        self.s_gripper.recv(4)

        if advanced:
            self.s_gripper.sendall(b"SET MOD 1\n")

        cowsay.cow("Calibrated to default settings!")

    def open(self):
        self.nonce_byte += 4
        self.s_gripper.sendall(b"SET POS 255\n")

    def close(self):
        # self.nonce_byte += 4
        self.s_gripper.sendall(b"SET GTO 1\n")
        self.s_gripper.recv(4)
        self.s_gripper.sendall(b"SET PR 255\n")
        self.s_gripper.sendall(b"SET POS 0\n")

    def get_pos(self):
        if self.nonce_byte > 0:
            self.s_gripper.recv(self.nonce_byte)
        self.nonce_byte = 0
        self.s_gripper.sendall(b"GET OBJ")
        return self.s_gripper.recv(4)

    def set_pos(self, pos: int):
        self.nonce_byte += 4
        self.s_gripper.sendall(b"SET POS %d\n" % pos)

    def send_custom_cmd(self, cmd: str, recv_byte: int = 0):
        self.s_gripper.sendall(cmd.encode())
        if recv_byte > 0:
            return self.s_gripper.recv(recv_byte)

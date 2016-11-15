import numpy as np
from RobotState import Action
import math

class MoveUpTestSource:
    def __init__(self, move_amount=0.05):
        self.move_amount = move_amount
        self.target = None

    def get_action(self, robot, robot_state):
        ret = Action()
        if self.target is None:
            self.target = robot_state.ee_trans.copy()
            self.target[0,3] += self.move_amount
        ret.twist_from_transform(robot_state, self.target)
        return ret

class SineTestSource:
    def __init__(self, move_amount=0.15, frequency=0.5):
        self.move_amount = move_amount
        self.target = None
        self.frequency = frequency
        self.t = 0.0
        self.z0 = 0.0

    def get_action(self, robot, robot_state):
        self.t += 1.0/60.0
        ret = Action()
        if self.target is None:
            self.target = robot_state.ee_trans.copy()
            self.z0 = self.target[0,3]
        self.target[0,3] = self.z0 + math.sin(self.t*self.frequency)*self.move_amount
        ret.twist_from_transform(robot_state, self.target)
        return ret

class SquareTestSource:
    def __init__(self, move_amount=0.02, frequency=2.0):
        self.move_amount = move_amount
        self.target = None
        self.frequency = frequency
        self.t = 0.0
        self.z0 = 0.0
        self.sign = 1.0

    def get_action(self, robot, robot_state):
        self.t += 1.0/60.0
        if self.t > self.frequency:
            self.sign *= -1.0
            self.t -= self.frequency
        ret = Action()
        if self.target is None:
            self.target = robot_state.ee_trans.copy()
            self.z0 = self.target[2,3]
        self.target[2,3] = self.z0 + self.sign*self.move_amount
        ret.twist_from_transform(robot_state, self.target)
        return ret
#RobotState.py

#keeps track of the state of the robot

import copy
import numpy as np
import rospy

from Utils import *

#TODO make this dynamic
NUM_FINGER_DOFS = rospy.get_param('/ada/num_finger_dofs', 2)
  
class RobotState(object):
  def __init__(self, ee_trans, finger_dofs, mode=0, num_modes=2):
    self.ee_trans = ee_trans.copy()
    self.finger_dofs = finger_dofs.copy()
    self.mode = mode
    self.num_modes = num_modes

  def get_pos(self):
    return self.ee_trans[0:3,3]

  def get_finger_dofs(self):
    return self.finger_dofs

  def switch_mode(self):
    self.mode = self.next_mode()

  def next_mode(self):
    return (self.mode+1)%self.num_modes

  def set_mode(self, mode):
    assert mode >= 0 and mode <= self.num_modes
    self.mode = mode

  def mode_after_action(self, action):
    if action.is_no_mode_switch():
      return self.mode
    else:
      return action.switch_mode_to

  def state_after_action(self, action, time):
    state_copy = copy.deepcopy(self)
    if not action.is_no_move():
      state_copy.ee_trans = ApplyTwistToTransform(action.move, state_copy.ee_trans, time)

    if not action.is_no_mode_switch():
      state_copy.mode = action.switch_mode_to

    return state_copy

  def num_finger_dofs(self):
    return len(finger_dofs)

#actions we can enact on the state
#corresponds to a mode switch and a twist
class Action(object):
  no_mode_switch=-1
  no_move = np.zeros(6)
  no_finger_vel = np.zeros(NUM_FINGER_DOFS)
  def __init__(self, twist=no_move, finger_vel=no_finger_vel, switch_mode_to=no_mode_switch):
    self.twist = twist
    self.finger_vel = finger_vel
    self.switch_mode_to = switch_mode_to

  def as_tuple(self):
    return (self.twist, self.switch_mode_to)

  def is_no_action(self):
    return self.twist == self.no_move and self.switch_mode_to == self.no_mode_switch and self.finger_vel == no_finger_vel

  def __eq__(self, other): 
    return self.twist == other.twist and self.switch_mode_to == other.switch_mode_to and self.finger_vel == other.finger_vel

  def is_no_mode_switch(self):
    return self.switch_mode_to == self.no_mode_switch

  def is_no_move(self):
    return np.linalg.norm(self.twist) < 1e-10

  def is_no_finger_move(self):
    return np.linalg.norm(self.finger_vel) < 1e-10

  # return the parts of this actions move that can be
  # altered by the user given the current robot
#  def move_in_mode(self, mode):
#    if mode == 0:
#      return self.move[:3]
#    elif mode == 1:
#      return self.move[3:]
#    else:
#      return self.move
    
  def __str__(self):
    return 'twist:' + str(self.twist) + ' finger:' + str(self.finger_vel) + ' mode to:' + str(self.switch_mode_to)
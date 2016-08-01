#RobotState.py

#keeps track of the state of the robot

import copy
import numpy as np

from Utils import *


  
#TODO add fingers to state
class RobotState(object):
  def __init__(self, ee_trans, mode=0, num_modes=2):
    self.ee_trans = ee_trans.copy()
    self.mode = mode
    self.num_modes = num_modes

  def get_pos(self):
    return self.ee_trans[0:3,3]

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



#actions we can enact on the state
#corresponds to a mode switch and a twist
#TODO add finger actions
class Action(object):
  no_mode_switch=-1
  no_move = np.zeros(6)
  def __init__(self, twist=no_move, switch_mode_to=no_mode_switch):
    self.twist = twist
    self.switch_mode_to = switch_mode_to

  def as_tuple(self):
    return (self.twist, self.switch_mode_to)

  def is_no_action(self):
    return self.twist == self.no_move and self.switch_mode_to == self.no_mode_switch

  def __eq__(self, other): 
    return self.twist == other.twist and self.switch_mode_to == other.switch_mode_to

  def is_no_mode_switch(self):
    return self.switch_mode_to == self.no_mode_switch

  def is_no_move(self):
    return np.linalg.norm(self.twist) < 1e-10

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
    return str(self.twist) + ','+str(self.switch_mode_to)

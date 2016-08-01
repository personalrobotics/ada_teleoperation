import rospy
import numpy as np

from input_handlers.UserInputListener import UserInputData
#from HydraListener import *
from RobotState import *

translation_weightings = np.array([0.3, 0.3, 0.3])
angular_weightings = np.array([0.9, 0.9, 0.9])

#mapping from user input to action
#if we were to execute input via direct teleop
class UserInputMapper(object):
  def __init__(self, num_motion_modes=2, num_finger_modes=0):
    self.num_motion_modes = num_motion_modes
    self.num_finger_modes = num_finger_modes

  def input_to_action(self, user_input_data, robot_state):
    action_to_ret = Action()
    #if the first button is now being pressed, switch modes
    if user_input_data.button_changes[0] == 1:
      action_to_ret.switch_mode_to = robot_state.next_mode()
      return action_to_ret
    
    #distinguish between 2d and 3d control modes

    curr_robot_mode = robot_state.mode
    if self.num_motion_modes == 2:
      if curr_robot_mode == 0:
        action_to_ret.twist[:3] = user_input_data.axes * translation_weightings
      else:
        ee_rot = robot_state.ee_trans[0:3,0:3]
        action_to_ret.twist[3:] = np.dot(ee_rot, (user_input_data.axes * angular_weightings))

    elif self.num_motion_modes == 3:
      if curr_robot_mode == 0:
        action_to_ret.twist[:2] = user_input_data.axes[:2] * translation_weightings[:2]
        #action_to_ret.move[0] *= -1.
      elif curr_robot_mode == 1:
        action_to_ret.twist[2] = user_input_data.axes[1] * translation_weightings[2]
        ee_rot = robot_state.ee_trans[0:3,0:3]
        rot_velocity = np.array([0, 0, user_input_data.axes[0]])
        action_to_ret.twist[3:] = np.dot(ee_rot, rot_velocity * angular_weightings)
      else:
        ee_rot = robot_state.ee_trans[0:3,0:3]
        rot_velocity = np.array([user_input_data.axes[1], user_input_data.axes[0], 0.])
        action_to_ret.twist[3:] = np.dot(ee_rot, (rot_velocity * angular_weightings))

    return action_to_ret

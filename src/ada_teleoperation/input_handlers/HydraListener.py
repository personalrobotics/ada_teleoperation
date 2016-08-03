import rospy
from razer_hydra.msg import Hydra, HydraPaddle
import numpy as np

from UserInputListener import *


class HydraListener(UserInputListener):
  def __init__(self):
    self.input_topic_name = "/hydra_calib"
    self.input_message_type = Hydra
    super(HydraListener, self).__init__()

  def message_to_data(self, message):
    right_ind = message.RIGHT
    left_ind = message.LEFT

    right_joy = message.paddles[right_ind].joy
    left_joy = message.paddles[left_ind].joy
    right_trigger = message.paddles[right_ind].trigger
    #left_trigger = message.paddles[left_ind].trigger

    axes = np.array([-right_joy[0], -right_joy[1], -left_joy[1], -left_joy[0], right_trigger])

    buttons = np.array([message.paddles[right_ind].buttons[1], message.paddles[right_ind].buttons[2]], dtype=int)
    #mode_switch_button = message.paddles[right_ind].buttons[1]
    #assist_switch_button = message.paddles[right_ind].buttons[2]

    return UserInputData(axes, buttons)



  #rotates the translation inputs to the correct world frame, applies weighting
  def translation_input_conversion(self, inputs, robot_state):
    #inputs_rotated = [inputs[0], i
    return inputs * translation_weightings


  #puts the rotation input into the world frame, applies weighting
  def rotation_input_conversion(self, inputs, robot_state):
    inputs_rotated = [-inputs[1], -inputs[0], inputs[2]]
    ee_rot = robot_state.ee_trans[0:3,0:3]
    return np.dot(ee_rot, inputs_rotated * angular_weightings)



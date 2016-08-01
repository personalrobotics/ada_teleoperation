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

    axes = np.array([right_joy[0], right_joy[1], left_joy[0], left_joy[1], right_trigger])

    buttons = np.array([message.paddles[right_ind].buttons[1], message.paddles[right_ind].buttons[2]])
    #mode_switch_button = message.paddles[right_ind].buttons[1]
    #assist_switch_button = message.paddles[right_ind].buttons[2]

    return UserInputData(axes, buttons)



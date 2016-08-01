import rospy
from razer_hydra.msg import Hydra, HydraPaddle
import numpy as np

from UserInputListener import *


class HydraListener(UserInputListener):
  def __init__(self):
    super(HydraListener, self).__init__()

  def init_listener(self):
    #rospy.init_node('hydra_listener')#, anonymous=True)

    rospy.Subscriber("/hydra_calib", Hydra, self.callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

  def message_to_data(self, message):
    right_ind = message.RIGHT
    left_ind = message.LEFT

    right_joy = message.paddles[right_ind].joy
    left_joy = message.paddles[left_ind].joy
    right_trigger = message.paddles[right_ind].trigger
    #left_trigger = message.paddles[left_ind].trigger
    mode_switch_button = message.paddles[right_ind].buttons[1]
    assist_switch_button = message.paddles[right_ind].buttons[2]

    move_velocity = np.array([right_joy[0], right_joy[1], left_joy[1]])
    close_hand_velocity = right_trigger

    return UserInputData(move_velocity, close_hand_velocity, mode_switch_button, assist_switch_button)



#class HydraData(object):
#  def __init__(self, message=None):
#    if message is None:
#      self.move_velocity = np.zeros(3)
#      self.close_hand_velocity = 0.
#      self.mode_switch_button = False
#    else:
#      right_ind = message.RIGHT
#      left_ind = message.LEFT
#
#      right_joy = message.paddles[right_ind].joy
#      left_joy = message.paddles[left_ind].joy
#      right_trigger = message.paddles[right_ind].trigger
#      #left_trigger = message.paddles[left_ind].trigger
#      mode_switch_button = message.paddles[right_ind].buttons[1]
#      assist_switch_button = message.paddles[right_ind].buttons[2]
#
#      self.move_velocity = np.array([right_joy[0], right_joy[1], left_joy[1]])
#      self.close_hand_velocity = right_trigger
#      self.mode_switch_button = mode_switch_button
#      self.assist_switch_button = assist_switch_button




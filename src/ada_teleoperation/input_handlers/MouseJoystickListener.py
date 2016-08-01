import rospy
from sensor_msgs.msg import Joy
import numpy as np

from UserInputListener import *

rosmsg_topic = "/ada/joy"


class MouseJoystickListener(UserInputListener):
  def __init__(self):
    self.input_topic_name = "/ada/joy"
    self.input_message_type = Joy
    super(MouseJoystickListener, self).__init__()


  def message_to_data(self, message):
    return UserInputData(np.array(message.axes), np.array(message.buttons))



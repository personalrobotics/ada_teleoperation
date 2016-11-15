import json
import numpy as np
from RobotState import Action
import math
import rospy
from std_msgs.msg import String
from threading import Lock # not sure if needed?

MAX_GRIP_VEL = 1.35

def clamp()

class ROSJSONSource:
  def __init__(self, topic_name):
    self.topic_name = topic_name
    self.init_listener(self.topic_name)
    self.input_lock = Lock()
    self.parsed_data = None

  def ros_callback(self, data):
    with self.input_lock:
      self.data = data

  def init_listener(self, topic_name):
    self.subscriber = rospy.Subscriber(topic_name, String,
                                       self.ros_callback)

  def parse_data(self):
    with self.input_lock:
      if self.data is not None:
        self.parsed_data = json.loads(self.data)
        self.data = None
    return self.parsed_data 

  def get_action(self, robot, robot_state):
    data = self.parse_data()
    ret = Action()

    if data is None:
      return ret

    speed = np.clip(data.get('speed', 1.0), 0.0, 2.0)
    fingervel = MAX_GRIP_VEL * np.clip(data.get('grip_vel', 0.0), -1.0, 1.0)

    if data.get('enabled', False):
      target = np.array(data.pose).reshape((4,4)).transpose()
      ret.twist_from_transform(robot_state, target, speed)
      ret.finger_vel[:] = fingervel

    return ret
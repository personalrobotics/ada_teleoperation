import json
import numpy as np
from RobotState import Action
import math
import rospy
from std_msgs.msg import String
from threading import Lock # not sure if needed?
import time

MAX_GRIP_VEL = 1.35
MIN_STATE_DT = 0.2

def serialize_robot_positions(robot):
  links = robot.GetLinks()
  return {l.GetName(): l.GetTransformPose().tolist() for l in links}

class ROSJSONSource:
  def __init__(self, in_topic_name = "vr_teleop", out_topic_name = "vr_robotstate"):
    self.init_listener(in_topic_name)
    self.init_publisher(out_topic_name)
    self.input_lock = Lock()
    self.parsed_data = None
    self.last_state_time = time.time()
    self.data = None

  def ros_callback(self, data):
    with self.input_lock:
      self.data = data.data

  def init_listener(self, topic_name):
    self.subscriber = rospy.Subscriber(topic_name, String,
                                       self.ros_callback)

  def init_publisher(self, topic_name):
    if topic_name is None or topic_name == "":
      self.pub = None
      return
    self.pub = rospy.Publisher(topic_name, String, queue_size=1)

  def parse_data(self):
    with self.input_lock:
      if self.data is not None:
        self.parsed_data = json.loads(self.data)
        self.data = None
    return self.parsed_data

  def get_action(self, robot, robot_state):
    data = self.parse_data()
    ret = Action()

    self.send_state(robot)

    if data is None:
      return ret

    speed = np.clip(data.get('speed', 1.0), 0.0, 2.0)
    fingervel = MAX_GRIP_VEL * np.clip(data.get('grip_vel', 0.0), -1.0, 1.0)

    if data.get('enabled', False):
      target = np.array(data.pose).reshape((4,4)).transpose()
      ret.twist_from_transform(robot_state, target, speed)
      ret.finger_vel[:] = fingervel

    return ret

  def send_as_json(self, msg):
    self.pub.publish(json.dumps(msg))

  def send_state(self, robot):
    if self.pub is None:
      return

    dt = time.time() - self.last_state_time
    if dt < MIN_STATE_DT:
      return

    self.last_state_time = time.time()

    with robot.GetEnv():
      msg = {"links": serialize_robot_positions(robot)}
    self.send_as_json(msg)
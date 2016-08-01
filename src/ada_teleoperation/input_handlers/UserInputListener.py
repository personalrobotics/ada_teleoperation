import numpy as np
import threading
import time

from abc import ABCMeta, abstractmethod


class UserInputListener(object):
  __metaclass__ = ABCMeta

  def __init__(self):
    self.inputlock = threading.Lock()
    self.init_listener()
    self.last_message = None

  def callback(self, data):
    with self.inputlock:
      self.last_message = data
      
  @abstractmethod
  def init_listener(self):
    raise NotImplementedError("Must override init_listener")

  def get_last_cmd(self):
    while not self.last_message:
      time.sleep(0)

    with self.inputlock:
      return self.message_to_data(self.last_message)

  @abstractmethod
  def message_to_data(self, message):
    raise NotImplementedError("Must override message_to_data")


class UserInputData(object):
  def __init__(self, move_velocity=np.zeros(3), close_hand_velocity=0., mode_switch_button=False, assist_switch_button=False):
    self.move_velocity = move_velocity
    self.close_hand_velocity = close_hand_velocity
    self.mode_switch_button = mode_switch_button
    self.assist_switch_button = assist_switch_button




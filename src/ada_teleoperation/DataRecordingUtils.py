import numpy as np
import copy
import cPickle as pickle
import os.path
import glob
import time
import subprocess
import signal
import psutil

import logging
logging.basicConfig()
logger = logging.getLogger('ada_teleoperation')

import rospy, rospkg

#from RobotState import *
#from Goal import Goal

file_directory_default = rospkg.RosPack().get_path('ada_teleoperation') + '/trajectory_data'
filename_base_default = 'trajdata_'

next_fileind_to_check = 0

class TrajectoryData(object):
  def __init__(self, init_info=None, file_directory=None, filename_base=None):
    self.init_info = init_info
    self.end_info = None

    if file_directory:
      self.file_directory = file_directory
    else:
      self.file_directory = file_directory_default

    if filename_base:
      self.filename_base = filename_base
    else:
      self.filename_base = filename_base_default


    self.data_per_time_step = []

  def set_init_info(self, *args, **kwargs):
    self.init_info = (args, add_time_to_dict(kwargs))
  
  def set_end_info(self, *args, **kwargs):
    self.end_info = (args, add_time_to_dict(kwargs))

  def add_datapoint(self, *args, **kwargs):
    if len(args) > 0:
      logger.warning('Attempting to save trajectory item with unspecified key. Please specify a key for all arguments')
    #self.data_per_time_step.append(TrajectoryData_PerTimeStep(*args, **kwargs))
    self.data_per_time_step.append((args, add_time_to_dict(kwargs)))

  def tofile(self, file_directory=None, filename_base=None):
    if file_directory is None:
      file_directory = self.file_directory
    if filename_base is None:
      filename_base = self.filename_base

    #first make sure directory exists
    if not os.path.exists(file_directory):
      os.makedirs(file_directory)
      
    filename = get_next_filename(file_directory, filename_base)
    with open(filename, 'w') as f:
      pickle.dump(self, f)



def load_all_trajectorydata(file_directory=None, filename_base=None):
  if file_directory is none:
    file_directory = file_directory_default
  if filename_base is none:
    filename_base = filename_base_default
  dir_and_filename_base = os.path.join(file_directory, filename_base)

  all_filenames = glob.glob(dir_and_filename_base + '*.pckl')
  all_filenames.sort()
  return [load_trajectorydata_from_file(filename) for filename in all_filenames]

def load_trajectorydata_from_file(filename):
  return pickle.load(open(filename, 'r'))


def get_next_filename(file_directory, filename_base, file_type='.pckl'):
  dir_and_filename_base = os.path.join(file_directory, filename_base)
  global next_fileind_to_check
  while True:
    filename = dir_and_filename_base + str(next_fileind_to_check).zfill(3) + file_type
    if not os.path.isfile(filename):
      break
    next_fileind_to_check += 1

  return filename



def add_time_to_dict(dict):
  dict['timestamp'] = time.time()
  return dict


def start_rosbag(topics, file_directory=None, filename_base=None):
  if file_directory is None:
    file_directory = file_directory_default
  if filename_base is None:
    filename_base = filename_base_default
  filename = get_next_filename(file_directory, filename_base, '.bag')
  
  topics_str = str(topics)[1:-2]
  topics_str = topics_str.replace("'", "")
  topics_str = topics_str.replace(",", "")

  command = "rosbag record -O " + filename + ' ' + topics_str
  
  rosbag_process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=file_directory)
  return rosbag_process

def stop_rosbag(rosbag_process):
  terminate_process_and_children(rosbag_process)


def terminate_process_and_children(p):
  children = psutil.Process(p.pid).get_children(recursive=True)
  for child_p in children:
    child_p.send_signal(signal.SIGINT)
  p.send_signal(subprocess.signal.SIGINT)

#maintain the data for each time step
#class TrajectoryData_PerTimeStep(object):
#  def __init__(self, *args, **kwargs):
#    if len(args) > 0:
#      logger.warning('Attempting to save trajectory item with unspecified key. Please specify a key for all arguments')
#
#  def __init__(self, state, dof_values, user_input, user_action, assistance_action, prior_useraction_each_goal, prob_each_goal, time):
#    self.state = copy.copy(state)
#    self.dof_values = copy.copy(dof_values)
#    self.user_input = copy.copy(user_input)
#    self.user_action = copy.copy(user_action)
#    self.assistance_action = copy.copy(assistance_action)
#    self.prior_useraction_each_goal = copy.copy(prior_useraction_each_goal)
#    self.prob_each_goal = copy.copy(prob_each_goal)
#    self.time = time

import rospy
import numpy as np
import time

from input_handlers.UserInputListener import UserInputData
from input_handlers import KinovaJoystickListener, HydraListener, MouseJoystickListener
#from HydraListener import *
from RobotState import *
from UserInputMapper import UserInputMapper

import openravepy
import adapy
import prpy

CONTROL_HZ = 40.



class AdaTeleopHandler:
  def __init__(self, env, robot):
#      self.params = {'rand_start_radius':0.04,
#             'noise_pwr': 0.3,  # magnitude of noise
#             'vel_scale': 4.,   # scaling when sending velocity commands to robot
#             'max_alpha': 0.6}  # maximum blending alpha value blended robot policy 

      self.env = env
      self.robot = robot

      self.sim = robot.simulated
      self.manip = self.robot.arm
  

      #select which input device we will be using
      self.joystick_listener = HydraListener()
      #self.joystick_listener = KinovaJoystickListener()
      #self.joystick_listener = MouseJoystickListener()
      self.user_input_mapper = UserInputMapper(num_motion_modes=2, num_finger_modes=0)

      self.Init_Robot()


  def Init_Robot(self):
    if not self.sim:
      self.SwitchToVelocityController()

    #set the robot state we keep track of
    self.robot_state = RobotState(self.GetEndEffectorTransform(), num_modes=self.user_input_mapper.num_motion_modes + self.user_input_mapper.num_finger_modes)

  def GetEndEffectorTransform(self):
    return self.manip.GetEndEffectorTransform()


  def ExecuteAction(self, action):
    self.robot_state.mode = self.robot_state.mode_after_action(action)
    #self.robot_state = self.robot_state.state_after_action(action)
    self.execute_twist(action.twist)


  # NOTE: twist is stacked [cartesian angular]
  def execute_twist(self, twist):
      jointVels, twist_opt = prpy.util.ComputeJointVelocityFromTwist(self.robot, twist, objective=prpy.util.quadraticPlusJointLimitObjective)
      return self.execute_joint_velocities(jointVels)


  def execute_joint_velocities(self, joint_velocities):
    #make sure in velocity is in limits
    joint_vel_limits = self.manip.GetVelocityLimits()
    ratio = np.absolute(joint_velocities/joint_vel_limits)

    if np.max(ratio) > 0.95:
      joint_velocities /= np.max(ratio)/0.95

    #move_time = 1./float(CONTROL_HZ)
    self.manip.Servo(joint_velocities)
    #time.sleep(move_time)


  def execute_finger_velocities(self, finger_velocities):
    print 'TODO implement finger velocities'


  def ExecuteDirectTeleop(self):
    robot_state = self.robot_state
  
    time_per_iter = 1./CONTROL_HZ

    while True:
      start_time = time.time()
      robot_state.ee_trans = self.GetEndEffectorTransform()
      ee_trans = robot_state.ee_trans
      robot_dof_values = self.robot.GetDOFValues()
      
      user_input_raw = self.joystick_listener.get_most_recent_cmd()
      direct_teleop_action = self.user_input_mapper.input_to_action(user_input_raw, robot_state)

      self.ExecuteAction(direct_teleop_action)

      end_time=time.time()
      
      rospy.sleep( max(0., time_per_iter - (end_time-start_time)))




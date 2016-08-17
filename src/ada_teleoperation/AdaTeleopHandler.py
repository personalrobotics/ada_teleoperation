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

CONTROL_HZ = 50.

mouse_interface_name = 'mouse'
kinova_joy_interface_name = 'kinova'
hydra_interface_name = 'hydra'

possible_teleop_interface_names = [mouse_interface_name, kinova_joy_interface_name, hydra_interface_name]


class AdaTeleopHandler:
  def __init__(self, env, robot, teleop_interface, num_input_dofs, is_done_func=None):
#      self.params = {'rand_start_radius':0.04,
#             'noise_pwr': 0.3,  # magnitude of noise
#             'vel_scale': 4.,   # scaling when sending velocity commands to robot
#             'max_alpha': 0.6}  # maximum blending alpha value blended robot policy 

      self.env = env
      self.robot = robot

      self.sim = robot.simulated
      self.manip = robot.arm
      self.hand = robot.arm.hand

      num_finger_dofs = len(self.hand.GetIndices())
      Action.set_no_finger_vel(num_finger_dofs)

      if is_done_func is None:
        self.is_done_func = Is_Done_Func_Default
      else:
        self.is_done_func = is_done_func

      #number of different modes used for motion is related to how many dofs of input we have
      if num_input_dofs == 2:
        self.num_motion_modes = 3
      elif num_input_dofs == 3:
        self.num_motion_modes = 2
      else:
        raise Exception('Number of input dofs being used must be 2 or 3')

      #select which input device we will be using
      if teleop_interface == mouse_interface_name:
        self.joystick_listener = MouseJoystickListener()
      elif teleop_interface == kinova_joy_interface_name:
        self.joystick_listener = KinovaJoystickListener()
      elif teleop_interface == hydra_interface_name:
        self.joystick_listener = HydraListener()
      else:
        raise Exception('Teleop interface not specified. Please specify one of: ' + str(possible_teleop_interface_names))
        
      self.user_input_mapper = UserInputMapper(interface_listener=self.joystick_listener, num_motion_modes=self.num_motion_modes, num_finger_modes=1)

      self.Init_Robot()


  def Init_Robot(self):
    if not self.sim:
      self.robot.SwitchToTeleopController()

    #set the robot state we keep track of
    self.robot_state = RobotState(self.GetEndEffectorTransform(), self.hand.GetDOFValues(), num_modes=self.user_input_mapper.num_motion_modes + self.user_input_mapper.num_finger_modes)

  def GetEndEffectorTransform(self):
    return self.manip.GetEndEffectorTransform()


  def ExecuteAction(self, action):
    self.robot_state.mode = self.robot_state.mode_after_action(action)
    #self.robot_state = self.robot_state.state_after_action(action)
    self.execute_twist(action.twist)
    self.execute_finger_velocities(action.finger_vel)


  # NOTE: twist is stacked [cartesian angular]
  def execute_twist(self, twist):
      #jointVels, twist_opt = prpy.util.ComputeJointVelocityFromTwist(self.robot, twist)# objective=prpy.util.quadraticPlusJointLimitObjective)
      jointVels, twist_opt = prpy.util.ComputeJointVelocityFromTwist(self.robot, twist, objective=weightedQuadraticObjective)
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
    self.hand.Servo(finger_velocities)


  def ExecuteDirectTeleop(self):
    robot_state = self.robot_state
  
    time_per_iter = 1./CONTROL_HZ

#    from ada_assistance_policy.AssistancePolicyVisualizationTools import *
#    vis = VisualizationHandler()
#    from KinovaStudyHelpers import *
#
#    robot = self.robot
#    ee_trans_home = self.GetEndEffectorTransform().copy()
#
#    curr_config = self.robot.arm.GetDOFValues()
#    converted_config = ConvertArmConfigHandedness(robot, curr_config)
#    if converted_config is not None:
#      robot.arm.SetDOFValues(converted_config)
#      print 'before: ' + str(curr_config)
#      print 'after: ' + str(converted_config)

    while not self.is_done_func(self.env, self.robot):
      start_time = time.time()
      robot_state.ee_trans = self.GetEndEffectorTransform()
      robot_state.finger_dofs = self.manip.hand.GetDOFValues()
      #ee_trans = robot_state.ee_trans
      
      user_input_raw = self.joystick_listener.get_most_recent_cmd()
      direct_teleop_action = self.user_input_mapper.input_to_action(user_input_raw, robot_state)
      self.ExecuteAction(direct_teleop_action)
      
#      ee_trans_before = self.GetEndEffectorTransform().copy()
#      config_before = robot.arm.GetDOFValues()
#      converted_pose = ConvertEEPoseHandedness(self.robot.GetTransform(), robot_state.ee_trans)
#      vis.draw_hand_poses([robot_state.ee_trans,converted_pose], marker_ns='ee_axis')

      end_time=time.time()
      
      rospy.sleep( max(0., time_per_iter - (end_time-start_time)))



def Is_Done_Func_Default(env, robot):
  return False

def weightedQuadraticObjective(dq, J, dx, *args):
    """
    Quadratic objective function for SciPy's optimization that penalizes translation error more then rotation error
    @param dq joint velocity
    @param J Jacobian
    @param dx desired twist
    @return objective the objective function
    @return gradient the analytical gradient of the objective
    """
    error = (np.dot(J, dq) - dx)
    #for some reason, passing weights as an argument caused trouble, so now just hard coded
    #error *= error_weights
    error *= np.array([5., 5., 5., 1., 1., 1.])
    objective = 0.5 * np.dot(np.transpose(error), error)
    gradient = np.dot(np.transpose(J), error)
    return objective, gradient

#
#def ang_error_quats(q1, q2):
#    quat_between = openravepy.quatMultiply( openravepy.quatInverse(q1), q2)
#    print quat_between
#    w = min(quat_between[0], 1.-1e-10)
#    w = max(w, -(1.-1e-10))
#    return 2.*np.arccos(w)
#
#from prpy.util import *
#def ang_error_mats(m1, m2):
#    #return AngleBetweenRotations(m1, m2)
#    return ang_error_quats(openravepy.quatFromRotationMatrix(m1), openravepy.quatFromRotationMatrix(m2))
#

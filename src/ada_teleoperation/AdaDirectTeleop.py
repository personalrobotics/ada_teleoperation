#!/usr/bin/env python

import rospy
import numpy as np
import argparse

from UserInputMapper import UserInputMapper
from RobotState import RobotState
import input_handlers
from input_handlers import KinovaJoystickListener, HydraListener, MouseJoystickListener

from AdaTeleopHandler import *


VIEWER_DEFAULT = 'InteractiveMarker'
SIMULATE_DEFAULT = True

def Initialize_Adapy():
    """ Initializes robot and environment through adapy, using the specified environment path

    @param env_path path to OpenRAVE environment
    @return environment, robot
    """

    parser = argparse.ArgumentParser('Ada Assistance Policy')
    parser.add_argument('-s', '--sim', action='store_true', default=SIMULATE_DEFAULT,
                        help='simulation mode')
    parser.add_argument('-v', '--viewer', nargs='?', const=True, default=VIEWER_DEFAULT,
                        help='attach a viewer of the specified type')
    #parser.add_argument('--env-xml', type=str,
                        #help='environment XML file; defaults to an empty environment')
    parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
    args = parser.parse_args()
    #env_path = '/environments/tablewithobjects_assisttest.env.xml'
    adapy_args = {'sim':args.sim,
                  'attach_viewer':args.viewer,
                  #'env_path':env_path
                  }
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    #openravepy.misc.InitOpenRAVELogging();
    env, robot = adapy.initialize(**adapy_args)
    Init_Robot(robot)
    return env,robot

def Init_Robot(robot):
    robot.SetActiveDOFs(range(6))
    #self.robot.arm.hand.OpenHand()
    #if (self.sim):
    robot_pose = np.array([[1, 0, 0, 0.409],[0, 1, 0, 0.338],[0, 0, 1, 0.795],[0, 0, 0, 1]])
    robot.SetTransform(robot_pose)
    if (robot.simulated):
      #servo simulator params
      robot.arm.servo_simulator.period=1./200.

def Reset_Robot(robot):
  if robot.simulated:
    num_hand_dofs = len(robot.arm.hand.GetDOFValues())
    inds, pos = robot.configurations.get_configuration('home')
    with robot.GetEnv():
      robot.SetDOFValues(pos, inds)
      robot.arm.hand.SetDOFValues(np.ones(num_hand_dofs)*0.1)
  else:
    #robot.arm.hand.OpenHand()
    robot.arm.PlanToNamedConfiguration('home', execute=True)


if __name__ == "__main__":
    rospy.init_node('ada_teleoperation', anonymous = True)

    env,robot = Initialize_Adapy()
    Reset_Robot(robot)
    ada_teleop = AdaTeleopHandler(env, robot)
    ada_teleop.ExecuteDirectTeleop()


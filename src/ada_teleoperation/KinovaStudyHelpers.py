#Helper files for our kinova study

import rospy
import numpy as np
from prpy.tsr.rodrigues import *
import openravepy



robot_handedness = rospy.get_param('/ada/handedness', 'left')


#def convertJointConfigToLeftHanded(config):




#takes pose (in world coordinates) for a right handed configuration
#robot and converts to left handed
def ConvertEEPoseHandedness(robot_pose, ee_pose):
  pose_robot_frame = np.dot(np.linalg.inv(robot_pose), ee_pose)

  mirrored_pose = np.eye(4)
  n = np.array([1., 0., 0.])
  for i in range(4):
    d = pose_robot_frame[0:3, i]
    mirrored_pose[0:3, i] = d - 2*(np.dot(d, np.transpose(n)))* n

  #this transform will be left-handed instead of right. flip y
  mirrored_pose[0:3, 1] *= -1.

  return np.dot(robot_pose, mirrored_pose)


def ConvertArmConfigHandedness(robot, config):
  with robot:
    robot.arm.SetDOFValues(config)

    pose = robot.arm.GetEndEffectorTransform()
    pose_converted = ConvertEEPoseHandedness(robot.GetTransform(), pose)
    #config_converted = robot.arm.FindIKSolution(pose_converted, 0)

    #find many solutions, return the one closest to the specified configuration
    sols = robot.arm.FindIKSolutions(pose_converted, 0)
    errs = np.sum(np.abs(sols - config)**2, axis=-1)
  return sols[np.argmin(errs), :]


def AddConstraintBoxes(env, robot, handedness='right', name_base="constraint_boxes_", visible=False):
    #add a box behind the robot
    box_behind = openravepy.RaveCreateKinBody(env,'')
    box_behind.SetName(name_base + 'behind')
    box_behind.InitFromBoxes(np.array([[0.,0.,0., 0.4, 0.1, 1.0]]), visible)
    env.Add(box_behind)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[1,3] = 0.54
    if handedness == 'right':
        T[0,3] += 0.25
    else:
        T[0,3] -= 0.25
    #T[2,3] = 0.5
    box_behind.SetTransform(T)


    #import IPython
    #IPython.embed()

    #add a box above so we don't swing that way too high
    box_above = openravepy.RaveCreateKinBody(env,'')
    box_above.SetName(name_base + 'above')
    box_above.InitFromBoxes(np.array([[0.,0.,0., 0.5, 0.5, 0.1]]), visible)
    env.Add(box_above)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[0,3] += 0.25
    T[1,3] -= 0.25
    T[2,3] += 0.90
    box_above.SetTransform(T)


    box_left = openravepy.RaveCreateKinBody(env,'')
    box_left.SetName(name_base + 'left')
    box_left.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_left)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] += 0.9
    else:
        T[0,3] += 0.25
    T[1,3] = 0.25
    #T[2,3] = 0.5
    box_left.SetTransform(T)

    box_right = openravepy.RaveCreateKinBody(env,'')
    box_right.SetName(name_base + 'right')
    box_right.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_right)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] -= 0.25
    else:
        T[0,3] -= 0.9
    T[1,3] = 0.25
    #T[2,3] = 0.5
    box_right.SetTransform(T)
    







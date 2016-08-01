#!/usr/bin/env python

import rospy
import numpy as np

from UserInputMapper import UserInputMapper
from RobotState import RobotState
import input_handlers
from input_handlers import KinovaJoystickListener, HydraListener, MouseJoystickListener


if __name__ == "__main__":
    rospy.init_node('ada_teleoperation', anonymous = True)
    joy = KinovaJoystickListener()
    user_input_mapper = UserInputMapper()

    import time
    while True:
        user_input_raw = joy.get_most_recent_cmd()
        print user_input_mapper.input_to_action(user_input_raw, RobotState(np.eye(4)))
        time.sleep(0.05)




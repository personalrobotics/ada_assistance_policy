#!/usr/bin/env python

from AdaHandler import *
from HydraListener import *
import rospy
import IPython


if __name__ == "__main__":
  rospy.init_node('ada_assistance_policy', anonymous = True)
  ada_handler = AdaHandler()
  for i in range(2):
    ada_handler.Reset_Trial(randomize_goal_init=True)
    ada_handler.execute_policy(simulate_user=False, fix_magnitude_user_command=False)
  #ada_handler.execute_direct_teleop(simulate_user=False)

  IPython.embed()


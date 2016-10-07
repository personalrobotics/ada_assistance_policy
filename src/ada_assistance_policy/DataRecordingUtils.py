import numpy as np
import copy
import cPickle as pickle
import os.path
import glob

import rospy
import rospkg

#from Robot_State import Robot_State
from ada_teleoperation.RobotState import *
from Goal import Goal


file_directory = rospkg.RosPack().get_path('ada_assistance_policy') + '/trajectory_data'
filename_base = 'trajdata_'

next_fileind_to_check = 0

class TrajectoryData(object):
  def __init__(self, start_state, goals, intended_goal_ind):
    self.start_state = start_state
    self.goals = goals
    self.intended_goal_ind = intended_goal_ind

    self.data_per_time_step = []

  def add_datapoint(self, *args, **kwargs):
    self.data_per_time_step.append(TrajectoryData_PerTimeStep(*args, **kwargs))


  def print_positions(self):
    s = ''
    for d in self.data_per_time_step:
      s += str(d.state.get_pos()) + '  ' 

    print s

  def tofile(self):
    #first make sure directory exists
    if not os.path.exists(file_directory):
      os.makedirs(file_directory)
      
    dir_and_filename_base = file_directory + '/' + filename_base
    filename = get_next_filename(dir_and_filename_base)
    with open(filename, 'w') as f:
      pickle.dump(self, f)



def load_all_trajectorydata():
  dir_and_filename_base = file_directory + '/' + filename_base
  all_filenames = glob.glob(dir_and_filename_base + '*.pckl')
  return [load_trajectorydata_from_file(filename) for filename in all_filenames]

def load_trajectorydata_from_file(filename):
  return pickle.load(open(filename, 'r'))


def get_next_filename(dir_and_filename_base):
  global next_fileind_to_check
  while True:
    filename = dir_and_filename_base + str(next_fileind_to_check).zfill(3) + '.pckl'
    if not os.path.isfile(filename):
      break
    next_fileind_to_check += 1

  return filename





#maintain the data for each time step
class TrajectoryData_PerTimeStep(object):
  def __init__(self, state, dof_values, user_input, user_action, assistance_action, prior_useraction_each_goal, prob_each_goal, time):
    self.state = copy.copy(state)
    self.dof_values = copy.copy(dof_values)
    self.user_input = copy.copy(user_input)
    self.user_action = copy.copy(user_action)
    self.assistance_action = copy.copy(assistance_action)
    self.prior_useraction_each_goal = copy.copy(prior_useraction_each_goal)
    self.prob_each_goal = copy.copy(prob_each_goal)
    self.time = time

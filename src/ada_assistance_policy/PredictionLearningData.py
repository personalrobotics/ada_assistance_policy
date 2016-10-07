# What we process data into for continuous learning prediction

import copy
import cPickle as pickle


file_directory = '/home/sjavdani/pr/src/ada_assistance_policy/trajectory_data_processed'

filename_base = 'learning_data'
#filename_base_onetraj = 'learning_data_traj_'



class PredictionLearningData(object):
  def __init__(self, data_per_traj=[]):
    self.data_per_traj = data_per_traj
  
  def add_trajectory_data(self, data_one_traj):
    self.data_per_traj.append(data_one_traj)

  def to_file(self):
    dir_and_filename = file_directory + '/' + filename_base + '.pckl'
    with open(dir_and_filename, 'w') as f:
      pickle.dump(self, f)
  

class PredictionLearningDataOneTraj(object):
  def __init__(self):
    self.data_per_time_step = []

  def add_data_one_timestep(self, *args, **kwargs):
    self.data_per_time_step.append(PredictionLearningDataOneTimestep(*args, **kwargs))



class PredictionLearningDataOneTimestep(object):
  def __init__(self, A, B, x, u, f):
    self.A = copy.copy(A)
    self.B = copy.copy(B)
    self.x = copy.copy(x)
    self.u = copy.copy(u)
    self.f = copy.copy(f)

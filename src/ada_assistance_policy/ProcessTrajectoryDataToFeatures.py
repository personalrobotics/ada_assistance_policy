#!/usr/bin/env python

from DataRecordingUtils import *
from PredictionLearningData import *

import numpy as np




# compute the feature vectors we would use to predict actions for the entire trajectory
def traj_to_feature_vec(traj):
  intended_goal_ind = traj.intended_goal_ind


  #for now, only grab the first 3 indices of states and actions
  data_inds = range(0,3)
  processed_traj_dat = PredictionLearningDataOneTraj()
  for data_ind,data_timestep in enumerate(traj.data_per_time_step[:-1]):
    curr_mode = data_timestep.state.mode
    u_t = data_timestep.user_action.move_in_mode(curr_mode)[data_inds]
    x_t = data_timestep.state.ee_trans[0:3,3]
    A_t = np.eye(len(data_inds))
    B_t = np.eye(len(data_inds))

    #TODO estimate B in one operation, not per dim
    x_t_next = traj.data_per_time_step[data_ind+1].state.ee_trans[0:3,3]
    for i in range(len(data_inds)):
      B_t[i,i] = x_t_next[i] - x_t[i]


    goal = traj.goals[intended_goal_ind]
    dist_all_target_poses = [np.linalg.norm(x_t - p[0:3,3]) for p in goal.target_poses]
    min_dist = np.min(dist_all_target_poses)
    dist_per_dim = []
    for i in range(3):
      dist_per_dim.append(np.min([np.linalg.norm(x_t[i] - p[i,3]) for p in goal.target_poses]))


    #assistance_action_currmode = data_timestep.assistance_action.move_in_mode(curr_mode)
    #TODO: write code to compute the predicted user action
    #predicted_user_action_currmode = data_timestep.predicted_user_action.move_in_mode(curr_mode)

    #feat = np.append(user_action_currmode, assistance_action_currmode)#, predicted_user_action_currmode)
    f_t = np.append(u_t, dist_per_dim)#, predicted_user_action_currmode)
    f_t = np.append(f_t, dist_per_dim)

    processed_traj_dat.add_data_one_timestep(A_t, B_t, x_t, u_t, f_t)

  return processed_traj_dat





if __name__ == "__main__":
  all_traj_data = load_all_trajectorydata()
  all_traj_data = [traj_to_feature_vec(traj) for traj in all_traj_data]
  all_processed_data = PredictionLearningData(all_traj_data)
  all_processed_data.to_file()

  import IPython
  IPython.embed()

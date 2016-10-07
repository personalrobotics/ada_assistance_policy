import numpy as np
from Goal import *
#from Robot import Robot

class UserBot:
    def __init__(self, goals):
        self.goals = goals
        self.goal_num = 0;

        # scaling factors for generating commands
        self.position_scale_vector = 10.

        self.clip_norm_val = 0.25
        self.usr_cmd_dim = 3
        self.reset_noise_filter()

    def set_user_goal(self, goal_num):
#        num_goals = self.robot.world.num_goals();
#        if (goal_num >= num_goals):
#            raise Exception('Desired goal', goal_num, 
#                    'exceeds max goal number', num_goals-1)
        self.goal_num = goal_num  

    def get_usr_cmd(self, end_effector_trans, goal_pose=None):
        if goal_pose is None:
            goal_pose = self.goals[self.goal_num].pose
        pos_diff =  self.position_scale_vector*(goal_pose[0:3,3] - end_effector_trans[0:3,3])

        pos_diff_norm = np.linalg.norm(pos_diff)

        if (pos_diff_norm > self.clip_norm_val):
          pos_diff /= pos_diff_norm/self.clip_norm_val

        # add colored noise to the command
        usr_cmnd = pos_diff;
        usr_cmnd[0:2] *= -1.
        #usr_cmnd += self.noise_pwr*np.linalg.norm(usr_cmnd)*np.random.randn(self.usr_cmd_dim)  
        #usr_cmnd += self.correl_coeff.dot(self.white_noise_hist)
        #self.white_noise_hist = np.vstack([usr_cmnd, self.white_noise_hist[0:-1,:]])
    
        #cmd = cmd / np.linalg.norm(cmd)
        return usr_cmnd

    def reset_noise_filter(self, noise_pwr=0.3, hist_size=50):
        correl_coeff = np.arange(hist_size, 0, -1) # creates vector [10, 9, 8, ... 1]
        self.correl_coeff = (correl_coeff / np.sum(correl_coeff))*noise_pwr;
        self.white_noise_hist = noise_pwr*self.clip_norm_val\
                * np.random.randn(hist_size, self.usr_cmd_dim);
        self.noise_pwr = noise_pwr
        self.hist_size = hist_size






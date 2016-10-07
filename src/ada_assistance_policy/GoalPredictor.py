import numpy as np
import scipy.misc
import IPython

logsumexp = scipy.misc.logsumexp

class GoalPredictor(object):
  max_prob_any_goal = 0.99
  log_max_prob_any_goal = np.log(max_prob_any_goal)
  def __init__(self, goals):
    self.goals = goals
    self.log_goal_distribution = np.log((1./len(self.goals))*np.ones(len(self.goals)))


  def update_distribution(self, values, q_values):
    self.log_goal_distribution -= q_values - values
    self.normalize_log_distribution()

  def normalize_log_distribution(self):
    log_normalization_val = logsumexp(self.log_goal_distribution)
    self.log_goal_distribution -= log_normalization_val
    self.clip_prob()

  def clip_prob(self):
    if len(self.log_goal_distribution) <= 1:
      return
    #check if any too high
    max_prob_ind = np.argmax(self.log_goal_distribution)
    if self.log_goal_distribution[max_prob_ind] > self.log_max_prob_any_goal:
      #see how much we will remove from probability
      diff = np.exp(self.log_goal_distribution[max_prob_ind]) - self.max_prob_any_goal
      #want to distribute this evenly among other goals
      diff_per = diff/(len(self.log_goal_distribution)-1.)

      #distribute this evenly in the probability space...this corresponds to doing so in log space
      # e^x_new = e^x_old + diff_per, and this is formulate for log addition
      self.log_goal_distribution += np.log( 1. + diff_per/np.exp(self.log_goal_distribution))
      #set old one
      self.log_goal_distribution[max_prob_ind] = self.log_max_prob_any_goal

  def get_distribution(self):
    return np.exp(self.log_goal_distribution)

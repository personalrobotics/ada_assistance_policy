#Handles converting openrave items to generic assistance policy
from Goal import *
import GoalPredictor as GoalPredictor
from ada_teleoperation.RobotState import Action
from AssistancePolicy import *
from OpenraveUtils import *
import math
import time

ADD_MORE_IK_SOLS = False

class AdaAssistancePolicy:

  def __init__(self, goals):
    self.assist_policy = AssistancePolicy(goals)
    self.goal_predictor = GoalPredictor.GoalPredictor(goals)

    self.goals = goals

  def update(self, robot_state, user_action):
    self.assist_policy.update(robot_state, user_action)
    values,q_values = self.assist_policy.get_values()
    self.goal_predictor.update_distribution(values, q_values)
    self.robot_state = robot_state

  def get_action(self, goal_distribution = np.array([]), **kwargs):
    if goal_distribution.size == 0:
      goal_distribution = self.goal_predictor.get_distribution()

    assisted_action = Action(twist=self.assist_policy.get_assisted_action(goal_distribution, **kwargs), finger_vel=self.assist_policy.user_action.finger_vel, switch_mode_to=self.assist_policy.user_action.switch_mode_to)

    return assisted_action


  def get_blend_action(self, goal_distribution = np.array([]), **kwargs):
    if goal_distribution.size == 0:
      goal_distribution = self.goal_predictor.get_distribution()

    max_prob_goal_ind = np.argmax(goal_distribution)


    #check if we meet the confidence criteria which dictates whether or not assistance is provided
    #use the one from ancas paper - euclidean distance and some threshhold
    #if blend_confidence_function_euclidean_distance(self.robot_state, self.goals[max_prob_goal_ind]):
    if blend_confidence_function_prob_diff(goal_distribution):
      goal_distribution_all_max = np.zeros(len(goal_distribution))
      goal_distribution_all_max[max_prob_goal_ind] = 1.0
      assisted_action = Action(twist=self.assist_policy.get_assisted_action(goal_distribution_all_max, **kwargs), finger_vel=self.assist_policy.user_action.finger_vel, switch_mode_to=self.assist_policy.user_action.switch_mode_to)
      return assisted_action
    else:
      #if we don't meet confidence function, use direct teleop
      return self.assist_policy.user_action

def blend_confidence_function_prob_diff(goal_distribution, prob_diff_required=0.4):
  if len(goal_distribution) <= 1:
    return True

  goal_distribution_sorted = np.sort(goal_distribution)
  return goal_distribution_sorted[-1] - goal_distribution_sorted[-2] > prob_diff_required

  manip_pos = robot_state.get_pos()
  goal_poses = goal.target_poses
  goal_pose_distances = [np.linalg.norm(manip_pos - pose[0:3,3]) for pose in goal_poses]
  dist_to_nearest_goal = np.min(goal_pose_distances)
  return dist_to_nearest_goal < distance_thresh

def blend_confidence_function_euclidean_distance(robot_state, goal, distance_thresh=0.10):
  manip_pos = robot_state.get_pos()
  goal_poses = goal.target_poses
  goal_pose_distances = [np.linalg.norm(manip_pos - pose[0:3,3]) for pose in goal_poses]
  dist_to_nearest_goal = np.min(goal_pose_distances)
  return dist_to_nearest_goal < distance_thresh

#generic functions
def goal_from_object(object, manip):
  pose = object.GetTransform()
  robot = manip.GetRobot()
  env = robot.GetEnv()

  #generate TSRs for object
  if not 'bowl' in object.GetName():
    target_tsrs = GetTSRListForObject(object, manip)

  #turn TSR into poses
  num_poses_desired = 30
  max_num_poses_sample = 500

  target_poses = []
  target_iks = []
  num_sampled = 0
  while len(target_poses) < num_poses_desired and num_sampled < max_num_poses_sample:
    print 'name: ' + object.GetName() + ' currently has ' + str(len(target_poses)) + ' goal poses'
    if not 'bowl' in object.GetName():
      num_sample_this = int(math.ceil(num_poses_desired/len(target_tsrs)))
      num_sampled += num_sample_this
      target_poses_idenframe = SampleTSRList(target_tsrs, num_sample_this)
      target_poses_tocheck = [np.dot(object.GetTransform(), pose) for pose in target_poses_idenframe]
    else:
      num_sample_this = num_poses_desired
      num_sampled += num_sample_this
      target_poses_tocheck = get_bowl_poses(object, num_samples_pose=num_sample_this, ee_offset=0.15)
    for pose in target_poses_tocheck:
      #check if solution exists
#      ik_sols = manip.FindIKSolutions(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
#      if len(ik_sols) > 0:

      
      #sample some random joint vals
      
#      lower, upper = robot.GetDOFLimits()
#      dof_vals_before = robot.GetActiveDOFValues()
#      dof_vals = [ np.random.uniform(lower[i], upper[i]) for i in range(6)]
#      robot.SetActiveDOFValues(dof_vals)
#      pose = manip.GetEndEffectorTransform()
#      robot.SetActiveDOFValues(dof_vals_before)

      ik_sol = manip.FindIKSolution(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if ik_sol is not None:
        if ADD_MORE_IK_SOLS:
          #get bigger list of ik solutions
          ik_sols = manip.FindIKSolutions(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if ik_sols is None:
            ik_sols = list()
          else:
            ik_sols = list(ik_sols)
          #add the solution we found before
          ik_sols.append(ik_sol)
        else:
          #if we don't want to add more, just use the one we found
          ik_sols = [ik_sol]
        #check env col
        target_poses.append(pose)
        target_iks.append(ik_sols)
#        with robot:
#          manip.SetDOFValues(ik_sol)
#          if not env.CheckCollision(robot):
#            target_poses.append(pose)
        if len(target_poses) >= num_poses_desired:
          break

  return Goal(pose, target_poses, target_iks)

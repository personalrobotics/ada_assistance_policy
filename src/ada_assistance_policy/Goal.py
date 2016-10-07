import numpy as np
import scipy

import tf
import tf.transformations as transmethods

from Utils import *

# Goals for prediction and assistance
# pose corresponds to the pose of the goal object
# target poses corresponds to all the grasp locations for this object

class Goal: 
    
    def __init__(self, pose, target_poses = list(), target_iks = list()):
      self.pose = pose
      self.pos = pose[0:3,3]

      if not target_poses:
        target_poses.append(pose)

      #copy the targets
      self.target_poses = list(target_poses)
      self.target_iks = list(target_iks)

      self.compute_quaternions_from_target_poses()

      #print 'NUM POSES: ' + str(len(self.target_poses))

    def compute_quaternions_from_target_poses(self):
      self.target_quaternions = [transmethods.quaternion_from_matrix(target_pose) for target_pose in self.target_poses]
    
    def at_goal(self, end_effector_trans):
      for pose,quat in zip(self.target_poses, self.target_quaternions):
        pos_diff =  pose[0:3,3] - end_effector_trans[0:3,3]
        trans_dist = np.linalg.norm(pos_diff)

        quat_dist = QuaternionDistance(transmethods.quaternion_from_matrix(end_effector_trans), quat)

        if (trans_dist < 0.01) and (quat_dist < np.pi/48):
          return True
      # if none of the poses in target_poses returned, then we are not at goal
      return False



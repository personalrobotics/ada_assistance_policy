#Utilities for dealing with openrave

import numpy as np
#import math
#import time
#import IPython
#import os, sys
#
#import tf
#import tf.transformations as transmethods
#import rospkg
#import rospy

import openravepy
import adapy
import prpy
import tf.transformations as transmethods

from prpy.tsr import *
from prpy.tsr.rodrigues import *
from prpy.tsr.tsr import *
def GetCylinderTSRs(radius, height, manip, T0_w = np.eye(4), Tw_e = np.eye(4), lateral_tolerance = 0.02):
    """
    Generates a tsr that is appropriate for grasping a cylinder.
        
    @param radius - The radius of the cylinder
    @param height - The height along the cylinder for performing the grasp
    @param manip - The manipulator to use for the grasp
    @param T0_w - A transform from the cylinder frame to the world frame
    @param lateral_tolerance - The tolerance for the height at which to grab the cylinder
    """

    with manip.GetRobot():
        manip.SetActive()
        manipidx= manip.GetRobot().GetActiveManipulatorIndex()

    Bw = np.array([[  .0,    .0],
                      [  .0,    .0],
                      [-lateral_tolerance,lateral_tolerance],
                      [  .0,    .0],   
                      [  .0,    .0],   
                      [ -np.pi,    np.pi]])
    cylinder_tsrs = []
    Tw_ee = Tw_e.copy()
    #Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3], rodrigues([-np.pi/2, 0, 0]))

    #Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3],np.dot(rodrigues([-np.pi/2, 0, 0]), rodrigues([0, 0, -np.pi/2.])))
    #Tw_ee[:3,3] = [0., radius, height]

    Tw_ee = numpy.array([[ 0., 0., 1., -radius], 
                        [ -1., 0.,  0., 0.], 
                        [ 0., -1.,  0., height], #glass_height
                        [ 0., 0.,  0., 1.]])


    cylinderTSR1 = TSR(T0_w=T0_w, Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
    #cylinder_tsrs.append(cylinderTSR1)

    Tw_ee = Tw_ee.copy()
    Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3], rodrigues([0., 0., np.pi]))
    #Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3],np.dot(rodrigues([-np.pi/2, 0, 0]), rodrigues([0, 0, np.pi/2.])))
    #Tw_ee[:3,3] = [0., radius, height]
    cylinderTSR2 = TSR(T0_w=T0_w, Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
    cylinder_tsrs.append(cylinderTSR2)

    return cylinder_tsrs


def GetTSRListForObject(obj, manip,
                              T0_w = np.eye(4),
                              Tw_e = np.eye(4),
                              ee_offset = -0.02): 
    """
    Returns a list of tsr chains that describe valid grasps
    for the object.
    
    @param obj - The object to be grasped.
    @param manip - The manipulator to perform the grasping
    @param T0_w - The transform from world to object frame
    @param Tw_e - The initial transform from end effector to object
    @param start_tsr - A flag indicating the tsr should be sampled
    for the start of the trajectory
    @param goal_tsr - A flag indicating the tsr should be sampled 
    for the end of the trajectory
    """

    # First grab the manipulator index, this is needed in the tsr specification
    with manip.GetRobot().GetEnv():
        with manip.GetRobot():
            manip.SetActive()
            manipidx = manip.GetRobot().GetActiveManipulatorIndex()

    # We will use the bounding box to attempt to infer the shape of the object
    with manip.GetRobot().GetEnv():
        with obj:
            # Assumption: the coordinate frame of the object is axis aligned
            identity_transform = np.eye(4)
            obj.SetTransform(identity_transform)
            obj_bb = obj.ComputeAABB()

    
    # first check if we have a cylinder
    # Assumption: Anything with approximately matching x and y dimensions is a cylinder
    lateral_tolerance = 0.02
    if np.abs(obj_bb.extents()[0] - obj_bb.extents()[1]) < 0.001:
        return GetTSRListForCylinderObject(obj, manip, T0_w, Tw_e, ee_offset, lateral_tolerance)
    else:
        return GetTSRListForBoxObject(obj, manip, T0_w, Tw_e, ee_offset, lateral_tolerance)

def GetTSRListForCylinderObject(obj, manip,
                              T0_w = np.eye(4),
                              Tw_e = np.eye(4),
                              ee_offset = 0.00,
                              lateral_tolerance=0.02):
 # First grab the manipulator index, this is needed in the tsr specification

    with manip.GetRobot().GetEnv():
        with manip.GetRobot():
            manip.SetActive()
            manipidx = manip.GetRobot().GetActiveManipulatorIndex()

    # We will use the bounding box to attempt to infer the shape of the object
    with manip.GetRobot().GetEnv():
        with obj:
            # Assumption: the coordinate frame of the object is axis aligned
            identity_transform = np.eye(4)
            obj.SetTransform(identity_transform)
            obj_bb = obj.ComputeAABB()

    radius = obj_bb.extents()[0] + ee_offset
    height = obj_bb.pos()[2]  # grasp in the middle
    cylinder_tsrs = GetCylinderTSRs(radius = radius,
                                  height = height,
                                  manip = manip,
                                  T0_w = T0_w,
                                  Tw_e = Tw_e,
                                  lateral_tolerance=lateral_tolerance)

    return cylinder_tsrs



def GetTSRListForBoxObject(obj, manip,
                              T0_w = np.eye(4),
                              Tw_e = np.eye(4),
                              ee_offset = 0.00,
                              lateral_tolerance=0.02):
 # First grab the manipulator index, this is needed in the tsr specification
    with manip.GetRobot().GetEnv():
        with manip.GetRobot():
            manip.SetActive()
            manipidx = manip.GetRobot().GetActiveManipulatorIndex()

    # We will use the bounding box to attempt to infer the shape of the object
    with manip.GetRobot().GetEnv():
        with obj:
            # Assumption: the coordinate frame of the object is axis aligned
            identity_transform = np.eye(4)
            obj.SetTransform(identity_transform)
            obj_bb = obj.ComputeAABB()


    # We have a box. Define a TSR for each side of the box. 
    #  Also, for each side define a TSR for two opposing end-effector orientations

    # This parameter defines the largest an edge can be for it to be graspable
    #   if one side is larger than this dimension, we won't add the associated
    #   TSRs to the chain list.
    max_spread = 0.2

    box_tsrs = []

    if 2.0*obj_bb.extents()[0] < max_spread:
        # negative y
        Bw = np.array([[  .0,    .0],
                          wists[  .0,    .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])

        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3], rodrigues([-np.pi/2, 0, 0]))
        Tw_ee[:3,3] += [0, -obj_bb.extents()[1] - ee_offset, 0.]
        boxTSR1 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR1)

        Bw = np.array([[  .0,    .0],
                          [  .0,   .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])

        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3],np.dot(rodrigues([-np.pi/2, 0, 0]), rodrigues([0, 0, np.pi])))
        Tw_ee[:3,3] += [0, -obj_bb.extents()[1] - ee_offset, 0.]
        boxTSR2 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR2)


        # positive y
        Bw = np.array([[  .0,    .0],
                          [  .0,    .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])
        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3], rodrigues([np.pi/2, 0, 0]))
        Tw_ee[:3, 3] += [0, obj_bb.extents()[1] + ee_offset, 0.]
        boxTSR3 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR3)

        Bw = np.array([[  .0,    .0],
                          [  .0,    .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])
        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3],np.dot(rodrigues([0, np.pi/2, 0]), rodrigues([0, 0, np.pi/2])))
        Tw_ee[:3, 3] += [0., obj_bb.extents()[1] + ee_offset, 0.]
        boxTSR4 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR4)

    if 2.0*obj_bb.extents()[1] < max_spread:
        # negative x
        Bw = np.array([[  .0,    .0],
                          [  .0,    .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])

        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3], rodrigues([-np.pi/2, 0, 0]))
        Tw_ee[:3,3] += [-obj_bb.extents()[0] - ee_offset, 0., 0.]
        boxTSR5 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR5)

        Bw = np.array([[  .0,    .0],
                          [  .0,   .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])

        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3],np.dot(rodrigues([-np.pi/2, 0, 0]), rodrigues([0, 0, np.pi])))
        Tw_ee[:3,3] += [-obj_bb.extents()[0] - ee_offset, 0., 0.]
        boxTSR6 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR6)


        # positive x
        Bw = np.array([[  .0,    .0],
                          [  .0,    .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])
        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3], rodrigues([np.pi/2, 0, 0]))
        Tw_ee[:3, 3] += [obj_bb.extents()[0] + ee_offset, 0., 0.]
        boxTSR7 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR7)

        Bw = np.array([[  .0,    .0],
                          [  .0,    .0],
                          [-lateral_tolerance, lateral_tolerance],
                          [  .0,    .0],   
                          [  .0,    .0],   
                          [  .0,    .0]])
        Tw_ee = Tw_e.copy()
        Tw_ee[:3,:3] = np.dot(Tw_ee[:3,:3],np.dot(rodrigues([0, np.pi/2, 0]), rodrigues([0, 0, np.pi/2])))
        Tw_ee[:3, 3] += [obj_bb.extents()[0] + ee_offset, 0., 0.]
        boxTSR8 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
        box_tsrs.append(boxTSR8)

    return box_tsrs



def SampleTSRList(tsr_list, num_samples_each):
  trans_samples = []
  for tsr in tsr_list:
    for i in range(num_samples_each):
      trans_samples.append(tsr.sample())
  return trans_samples


def get_bowl_poses(obj, ee_offset = 0.00, num_samples_pose=15):
    obj_transform = obj.GetTransform()

    obj_aabb = obj.ComputeAABB()

    #for bowl objects from here

    height = 2.*obj_aabb.extents()[2] #assuming zero is at base of object
    radius = obj_aabb.extents()[0]

    #rotate to get arm to point down
    #rotx = transmethods.rotation_matrix(-np.pi/2., np.array([1.,0.,0.]))
    rotz = transmethods.rotation_matrix(np.pi, np.array([0.,0.,1.]))

    hand_trans_base = np.eye(4)
    #hand_trans_base = np.dot(rotx, np.dot(rotx, hand_trans_base))

    #we will sample a point on top object by sampling a number from 0 to 2pi
    #and sampling a point on circle
    #hand will be rotated to keep 2 fingers outside
    poses = []
    for i in range(num_samples_pose):
        hand_trans = np.copy(hand_trans_base)

        rand_rad = np.random.uniform(0., 2.*np.pi)
        rand_rot = transmethods.rotation_matrix(rand_rad, np.array([0., 0., 1.]))

        hand_trans = np.dot(rand_rot, hand_trans)
        #half the time rotate so fingers opposite direction
        if np.random.uniform() < 0.5:
          hand_trans = np.dot(rotz, hand_trans)
        hand_trans[:3,:3] = np.dot(hand_trans[:3,:3],rodrigues([0., 0., np.pi/2]))
        hand_trans[0,3] += radius*np.cos(rand_rad)
        hand_trans[1,3] += radius*np.sin(rand_rad)
        hand_trans[2,3] += height + ee_offset


        poses.append(np.dot(obj_transform, hand_trans))
    return poses

#!/usr/bin/env python

#Sets up the robot, environment, and goals to use shared autonomy code for grasping objects

from AdaHandler import *
from ada_teleoperation.AdaTeleopHandler import possible_teleop_interface_names
from AdaAssistancePolicy import goal_from_object
import rospy
import rospkg
import IPython

import numpy as np

from functools import partial


import openravepy
import adapy
import prpy


VIEWER_DEFAULT = 'InteractiveMarker'

def Initialize_Adapy(args, env_path='/environments/tablewithobjects_assisttest.env.xml'):
    """ Initializes robot and environment through adapy, using the specified environment path

    @param env_path path to OpenRAVE environment
    @return environment, robot
    """
    #env_path = '/environments/tablewithobjects_assisttest.env.xml'
    adapy_args = {'sim':args.sim,
                  'attach_viewer':args.viewer,
                  'env_path':env_path
                  }
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    #openravepy.misc.InitOpenRAVELogging();
    env, robot = adapy.initialize(**adapy_args)
    Init_Robot(robot)
    return env,robot

def Init_Robot(robot):
    robot.SetActiveDOFs(range(6))
    #self.robot.arm.hand.OpenHand()
    #if (self.sim):
    robot_pose = np.array([[1, 0, 0, 0.409],[0, 1, 0, 0.338],[0, 0, 1, 0.795],[0, 0, 0, 1]])
    robot.SetTransform(robot_pose)
    if (robot.simulated):
      #servo simulator params
      robot.arm.servo_simulator.period=1./200.




def Initialize_Goals(env, robot, randomize_goal_init=False):
  while True:
    goals, goal_objects = Init_Goals(env, robot, randomize_goal_init)

    #check if we have sufficient poses for each goal, to see if initialization succeeded
    for goal in goals:
      if len(goal.target_poses) < 25:
        continue
    break

  return goals, goal_objects


def Init_Goals(env, robot, randomize_goal_init=False):
    goal_objects = []
    goal_objects.append(env.GetKinBody('glass'))
    goal_objects.append(env.GetKinBody('fuze_bottle'))
    #goal_objects.append(env.GetKinBody('bowl'))
    #goal_objects.append(env.GetKinBody('pop_tarts'))

    obj_pose= robot.GetTransform()
#    obj_pose[0,3] += 0.40
#    obj_pose[1,3] += -0.35
#    obj_pose[2,3] += -0.055
    obj_pose[0,3] += 0.50
    obj_pose[1,3] += -0.35
    obj_pose[2,3] += -0.005
    
    #sebastian values
#    obj_pose[0,3] -= 0.10
#    obj_pose[1,3] += -0.50
#    obj_pose[2,3] += 0.25
#

    if randomize_goal_init:
      obj_pose[0:3,3] += np.random.rand(3)*0.10 - 0.05
    goal_objects[0].SetTransform(obj_pose)

    obj_pose= robot.GetTransform()

    # sebastian values
#    obj_pose[0,3] += 0.20
#    obj_pose[1,3] += -0.50
#    obj_pose[2,3] += 0.22

    #obj_pose[2,3] += -0.06
    obj_pose[0,3] += 0.05
    obj_pose[1,3] += -0.50
    obj_pose[2,3] += -0.02
    if randomize_goal_init:
      obj_pose[0:3,3] += np.random.rand(3)*0.10 - 0.05
    goal_objects[1].SetTransform(obj_pose)



    obj_pose= robot.GetTransform()

    # sebastian values
#    obj_pose[0,3] += 0.20
#    obj_pose[1,3] += -0.50
#    obj_pose[2,3] += 0.22

    #obj_pose[2,3] += -0.06
#    obj_pose[0,3] += 0.30
#    obj_pose[1,3] += -0.60
#    obj_pose[2,3] += -0.02
#    if randomize_goal_init:
#      obj_pose[0:3,3] += np.random.rand(3)*0.10 - 0.05
#    goal_objects[2].SetTransform(obj_pose)

#      obj_pose= robot.GetTransform()
#      obj_pose[0,3] += 0.45
#      obj_pose[1,3] += -0.2
#      obj_pose[2,3] += -0.055
#      #obj_pose = np.dot(obj_pose, transmethods.rotation_matrix(np.pi/2., np.array([1., 0., 0.])))
#      goal_objects[2].SetTransform(obj_pose)

#      obj_pose= robot.GetTransform()
#      obj_pose[0,3] += 0.4
#      obj_pose[1,3] += -0.2
#      obj_pose[2,3] += -0.055
#      goal_objects[3].SetTransform(obj_pose)

    #disable collisions for IK
    for obj in goal_objects:
      obj.Enable(False)

    goals = Set_Goals_From_Objects(goal_objects, robot)

    #for obj in goal_objects:
    #  obj.Enable(True)

    return goals, goal_objects


def Set_Goals_From_Objects(goal_objects, robot):
  #construct filename where data might be
#    path_to_pkg = rospkg.RosPack().get_path('ada_assistance_policy')
#    filename = path_to_pkg + "/" + cached_data_dir + "/"
#    for obj in goal_objects:
#      filename += str(obj.GetName())
#      str_pos = str(obj.GetTransform()[0:3, -1])[2:-2]
#      str_pos = str_pos.replace(" ","")
#      str_pos = str_pos.replace(".","")
#      filename += str_pos
#    filename += '.pckl'
#
#    #see if cached, if not load and save
#    if os.path.isfile(filename) and not RESAVE_GRASP_POSES:
#      with open(filename, 'r') as file:
#        items = pickle.load(file)
#        goals = items['goals']
#    else:
  goals = []
  for obj in goal_objects:
    goals.append(goal_from_object(obj, robot.arm))
#      with open(filename, 'w') as file:
#        items = {}
#        items['goals'] = goals
#        pickle.dump(items, file)

  return goals


def Finish_Trial_Func(robot):
  if robot.simulated:
    num_hand_dofs = len(robot.arm.hand.GetDOFValues())
    robot.arm.hand.SetDOFValues(np.ones(num_hand_dofs)*0.8)
  else:
    robot.arm.hand.CloseHand()

def Reset_Robot(robot):
  if robot.simulated:
    num_hand_dofs = len(robot.arm.hand.GetDOFValues())
    inds, pos = robot.configurations.get_configuration('home')
    with robot.GetEnv():
      robot.SetDOFValues(pos, inds)
      robot.arm.hand.SetDOFValues(np.ones(num_hand_dofs)*0.1)
  else:
    robot.arm.hand.OpenHand()
    robot.arm.PlanToNamedConfiguration('home', execute=True)



if __name__ == "__main__":
  parser = argparse.ArgumentParser('Ada Assistance Policy')
  parser.add_argument('-s', '--sim', action='store_true', default=SIMULATE_DEFAULT,
                      help='simulation mode')
  parser.add_argument('-v', '--viewer', nargs='?', const=True, default=VIEWER_DEFAULT,
                      help='attach a viewer of the specified type')
  #parser.add_argument('--env-xml', type=str,
                      #help='environment XML file; defaults to an empty environment')
  parser.add_argument('--debug', action='store_true',
                      help='enable debug logging')
  parser.add_argument('-input', '--input-interface-name', help='name of the input interface. Possible choices: ' + str(possible_teleop_interface_names), type=str)
  parser.add_argument('-joy_dofs', '--num-input-dofs', help='number of dofs of input, either 2 or 3', type=int, default=2)
  args = parser.parse_args()

  rospy.init_node('ada_assistance_policy', anonymous = True)


  #find environment path
  path_to_pkg = rospkg.RosPack().get_path('ada_assistance_policy')
  env_path = os.path.join(path_to_pkg, 'data', 'environments', 'tablewithobjects_assisttest.env.xml')

  
  env,robot = Initialize_Adapy(args, env_path=env_path)

  finish_trial_func_withrobot = partial(Finish_Trial_Func, robot=robot)
  for i in range(1):
    Reset_Robot(robot)
    goals, goal_objects = Initialize_Goals(env, robot, randomize_goal_init=False)
    ada_handler = AdaHandler(env, robot, goals, goal_objects, args.input_interface_name, args.num_input_dofs, use_finger_mode=False)
    ada_handler.execute_policy(simulate_user=False, direct_teleop_only=False, fix_magnitude_user_command=False, finish_trial_func=finish_trial_func_withrobot)
  #ada_handler.execute_direct_teleop(simulate_user=False)

  IPython.embed()


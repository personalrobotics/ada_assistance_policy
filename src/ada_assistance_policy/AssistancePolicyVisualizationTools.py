from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import rospy
import numpy as np

#colormap
from matplotlib import cm

color_rbg_usercmd = [0.1, 0.7, 0.1]
color_rbg_assistcmd = [0.9, 0., 0.05]


class VisualizationHandler:

  def __init__(self):
    self.marker_publisher = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=50)



  def draw_probability_text_given_positions(self, positions, probabilities):
    all_markers = MarkerArray()
    for ind,(position,prob) in enumerate(zip(positions,probabilities)):
      marker = Marker()
      marker.header.frame_id = "map"
      marker.header.stamp = rospy.Time.now()
      marker.type = Marker.TEXT_VIEW_FACING
      marker.id = ind
      marker.ns = 'probabilities'
      marker.lifetime.secs = 1

      marker.pose.position.x = position[0]
      marker.pose.position.y = position[1]
      marker.pose.position.z = position[2] + 0.1
      marker.pose.orientation.x = 0.
      marker.pose.orientation.y = 0.
      marker.pose.orientation.z = 0.
      marker.pose.orientation.w = 1.

      marker.text = str(np.ceil(prob*100.)/100.)
      #marker.text = str(round(prob, 2))
      marker.scale.x = 0.09
      marker.scale.y = 0.09
      marker.scale.z = 0.09

      color_jet = cm.jet(prob)
      marker.color.r = color_jet[0]
      marker.color.g = color_jet[1]
      marker.color.b = color_jet[2]
      marker.color.a = color_jet[3]

#      for link in obj.GetLinks():
#        for geom in link.GetGeometries():
#          geom.SetDiffuseColor(np.array(color_jet))

      all_markers.markers.append(marker)

    self.marker_publisher.publish(all_markers)

  def draw_probability_text(self, object_poses, probabilities):
    positions = []
    for obj_pose in object_poses:
      positions.append( [obj_pose[0,3], obj_pose[1,3], obj_pose[2,3]+0.28])
    
    self.draw_probability_text_given_positions(positions, probabilities)

  #adds arrow markers to draw hand poses as arrows
  def draw_hand_poses(self, poses, marker_ns='axes'):
    all_markers = MarkerArray()
    for pose in poses:
      arrow_markers = pose_to_arrow_markers(pose, ns=marker_ns, id_start=len(all_markers.markers))
      for marker in arrow_markers:
        all_markers.markers.append(marker)
    self.marker_publisher.publish(all_markers)

  
  def draw_action_arrows(self, end_effector_trans, user_cmd, assist_cmd, scale_cmd_arrow=1.0):
    all_markers = MarkerArray()
    start_pt = end_effector_trans[0:3,3]

    end_pt_user = (start_pt + user_cmd[0:3]*scale_cmd_arrow)
    all_markers.markers.append(arrow_marker(start_pt, end_pt_user, color_rbg_usercmd, marker_id=0))

    end_pt_assist = (start_pt + assist_cmd[0:3]*scale_cmd_arrow)
    all_markers.markers.append(arrow_marker(start_pt, end_pt_assist, color_rbg_assistcmd, marker_id=1))
    self.marker_publisher.publish(all_markers)





def pose_to_arrow_markers(pose, ns='axes', id_start=0):
  markers = []
  scale_size = 0.1
  dims = [.01, .02, 0]
  dirs = []
  dirs.append(np.array([1.,0.,0.]))
  dirs.append(np.array([0.,1.,0.]))
  dirs.append(np.array([0.,0.,1.]))
  for ind,dir in enumerate(dirs):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.ARROW
    marker.id = id_start + ind
    marker.lifetime.secs = 1
    marker.ns = ns

    marker.scale.x = dims[0]
    marker.scale.y = dims[1]
    marker.scale.z = dims[2]

    marker.pose.position.x = 0.
    marker.pose.position.y = 0.
    marker.pose.position.z = 0.
    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.

 

    start_pt = pose[0:3,3]
    end_pt = np.dot(pose[0:3,0:3], dir)*scale_size + start_pt
    marker.points = [np_array_to_point(start_pt), np_array_to_point(end_pt)]

    marker.color.r = dir[0]
    marker.color.g = dir[1]
    marker.color.b = dir[2]
    marker.color.a = 1.0

    markers.append(marker)

  return markers

def arrow_marker(start_pt, end_pt, color_rbg, ns='axes', marker_id=0):
  scale_size = 0.1
  dims = [.01, .02, 0]
  marker = Marker()
  marker.header.frame_id = "map"
  marker.header.stamp = rospy.Time.now()
  marker.type = Marker.ARROW
  marker.id = marker_id
  marker.lifetime.secs = 1
  marker.ns = ns

  marker.scale.x = dims[0]
  marker.scale.y = dims[1]
  marker.scale.z = dims[2]

  marker.pose.position.x = 0.
  marker.pose.position.y = 0.
  marker.pose.position.z = 0.
  marker.pose.orientation.x = 0.
  marker.pose.orientation.y = 0.
  marker.pose.orientation.z = 0.
  marker.pose.orientation.w = 1.


  #start_pt = pose[0:3,3]
  #end_pt = np.dot(pose[0:3,0:3], dir)*scale_size + start_pt
  marker.points = [np_array_to_point(start_pt), np_array_to_point(end_pt)]

  marker.color.r = color_rbg[0]
  marker.color.g = color_rbg[1]
  marker.color.b = color_rbg[2]
  marker.color.a = 1.0

  return marker



def np_array_to_point(pt_np):
  pt = Point()
  pt.x = pt_np[0]
  pt.y = pt_np[1]
  pt.z = pt_np[2]
  return pt

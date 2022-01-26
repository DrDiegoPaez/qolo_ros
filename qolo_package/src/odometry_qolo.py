#! /usr/bin/env python3
#########  ROS version for publishing 2D pose from Intel T265 Odometry ##########
##### Author: Diego F. Paez G.
##### Data: 2021/03/10

import time
import math
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension 
import tf2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion, Twist, Vector3
from scipy.interpolate import UnivariateSpline

#import matplotlib.pyplot as plt
# Fast Clipper Function
clipper = lambda x, l, u: l if x < l else u if x > u else x

pose_qolo = Pose2D()
pose_pub = None
pose_t265 = Pose2D()
# -0.126 0.0 -0.235
pose_t265.x = -0.126
pose_t265.y = 0
pose_t265.theta = 0

# class myNode:
#     def __init__(self, *args):
#         self.tf_listener_ = TransformListener()

#     def example_function(self):
#         if self.tf.frameExists("/base_link") and self.tf.frameExists("/fingertip"):
#             # t = self.tf_listener_.getLatestCommonTime("/tf_qolo", "/fingertip")
#             p1 = geometry_msgs.msg.Pose2D()
#             p1.header.frame_id = "fingertip"
#             p1.pose.orientation.w = 1.0    # Neutral orientation
#             p_in_base = self.tf_listener_.transformPose("/tf_qolo", p1)
#             print "Position of the fingertip in the robot base:"
#             print p_in_base


def get_pose(odom_data):
  global pose_qolo, pose_t265
  # rot = np.zeros((4,))
  # tf_qolo = PoseStamped()
  # print Odometry.pose.pose
  # (trans, rot) = tf_listener.lookupTransform('/t265_pose_frame', '/tf_qolo', rospy.Time(0))
  # tf_qolo.pose = odom_data.pose.pose
  # tf_qolo = tf_listener.transformPose("/tf_qolo",tempPose)
  # print ("t265 = ", odom_data.pose.pose.position.x, odom_data.pose.pose.position.y)
  pose_qolo.x = odom_data.pose.pose.position.x
  pose_qolo.y = odom_data.pose.pose.position.y
  # rot[0] = odom_data.pose.pose.orientation.x
  # rot[1] = odom_data.pose.pose.orientation.y
  # rot[2] = odom_data.pose.pose.orientation.z
  # rot[3] = odom_data.pose.pose.orientation.w
  # pose_qolo.theta = rpy[2]
  pose_qolo.theta = tf.transformations.euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y,odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])[2]
  # print ("pose = ", pose_qolo.x, pose_qolo.y, pose_qolo.theta)
  return


def main():
  global pose_pub, pose_qolo, pose_t265

  # pose_sub = rospy.Subscriber("qolo/pose2D", Pose2D, pose_callback, queue_size=1)
   
  rospy.init_node('qolo_odom', anonymous=True)
  rate = rospy.Rate(200) #  100 [Hz]
  pose_pub = rospy.Publisher('qolo/pose2D',Pose2D, queue_size=1) 
  # Example call for subscriber
  # pose_sub = rospy.Subscriber("qolo/pose2D", Pose2D, pose_callback, queue_size=1)
  odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, get_pose)
  while not rospy.is_shutdown():
    pose_pub.publish(pose_qolo)
    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
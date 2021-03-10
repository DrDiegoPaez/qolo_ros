#! /usr/bin/env python3
#########  ROS version of Trajectory Tracking with safety ##########
##### Author: Diego F. Paez G.
##### Data: 2021/03/10

import time
import math
import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension 

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Quaternion, Twist, Vector3

from scipy.interpolate import UnivariateSpline

#import matplotlib.pyplot as plt

# Fast Clipper Function
clipper = lambda x, l, u: l if x < l else u if x > u else x

dx_prev = np.array([[0.0], [0.0]])
dx = np.array([[0.0], [0.0]])

pose_qolo = pose2D()
pose_pub = None

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
	global pose_qolo
	tempPose = PoseStamped()
	# print Odometry.pose.pose
	# (trans, rot) = tf_listener.lookupTransform('/t265_pose_frame', '/tf_qolo', rospy.Time(0))
	tempPose.pose = odom_data.pose.pose
    tf_qolo = tf_listener.transformPose("/tf_qolo", tempPose)

    pose_qolo[0] = tf_qolo.pose.position.x
    pose_qolo[1] = tf_qolo.pose.position.y
	quaternion[0] = tf_qolo.pose.orientation.x
    quaternion[1] = tf_qolo.pose.orientation.y
	quaternion[2] = tf_qolo.pose.orientation.z
    quaternion[3] = tf_qolo.pose.orientation.w
	rpy = tf.transformations.euler_from_quaternion(quaternion)
    pose_qolo[2] = rpy[2]
   	print ("pose = ", pose_qolo[0], pose_qolo[1], pose_qolo[2])
   	return


def main():
   global pose_pub, pose_qolo

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
   main()
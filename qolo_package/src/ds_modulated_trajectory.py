#! /usr/bin/env python3
#########  ROS version of Trajectory Tracking with safety ##########
##### Author: Diego F. Paez G.
##### Data: 2020/05/18

import time
import math
import rospy
# from rds_network_ros.srv import * #VelocityCommandCorrectionRDS

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension 
from geometry_msgs.msg import Pose2D

import numpy as np
from scipy.interpolate import UnivariateSpline
import dynamical_system_representation as ds
#import matplotlib.pyplot as plt

# Fast Clipper Function
clipper = lambda x, l, u: l if x < l else u if x > u else x

dx_prev = np.array([[0.0], [0.0]])
dx = np.array([[0.0], [0.0]])

DEBUG_FLAG = False
MaxSpeed = 1.5/2 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
MaxAngular = 4.124/4
D_angular = 10
D_linear = 10

ref_vel = 0.5
# control_point = 0.9      ## WHY 0.9 ??
# control_pt = [0.5275, 0.]
control_pt = [0.7, 0.]
stop_distance = 0.5
time_limit = 90

# Setting the Attractor at the origin --> Requires starting the robot's odometry with the desired offset instead
Attractor = np.array([[0.0 + control_pt[0]], [0.0+control_pt[1]]])

pose = [0., 0., 0.]
command_publisher = None
previous_command_linear = None
previous_command_angular = None
data_remote = Float32MultiArray()

def pose_callback(data):
   global pose
   pose[0] = data.x
   pose[1] = data.y
   pose[2] = data.theta

def control_pt_callback(data):
   global control_pt
   control_pt[0] = data.x
   control_pt[1] = data.y

def ds_generation(x,y,phi):
   global dx_prev, dx, ref_vel
   try:
      # x = 1
      # y = 1
      # phi = 0.79
      Ctime = time.clock()
      translation = np.array([[x], [y]])
      Rotation = np.array([
               [np.cos(phi), -np.sin(phi)],
               [np.sin(phi),  np.cos(phi)]])
      p_ref_local = np.array([[control_pt[0]], [control_pt[1]]])
      p_ref_global = np.matmul(Rotation, p_ref_local) + translation

      if DEBUG_FLAG:
         print(" Attractor X, Y = ",Attractor[0,0],Attractor[1,0])
         print(" Current X, Y, Phi = ",p_ref_global[0,0],p_ref_global[1,0], np.rad2deg(phi))

      dx = ds.linearAttractor_const(p_ref_global, Attractor, ref_vel, stop_distance)
      ## Call here Modulated DS
      # dx = ds.modulation( ) 

      v_command_p_ref_global = dx
      
      if DEBUG_FLAG:
         print(" V_global2 = ",v_command_p_ref_global[0,0],v_command_p_ref_global[1,0])

      J_p_ref_inv = np.array([
                     [1.0, p_ref_local[1,0]/p_ref_local[0,0]],
                     [0.0, 1/p_ref_local[0,0]]])
      
      v_command_p_ref_local = np.matmul(np.transpose(Rotation), v_command_p_ref_global)
      if DEBUG_FLAG:
         print(" V_local = ",v_command_p_ref_local[0,0],v_command_p_ref_local[1,0])

      cammand_robot = np.matmul(J_p_ref_inv, v_command_p_ref_local)

      command_angular = clipper(cammand_robot[1,0], -MaxAngular, MaxAngular)
      command_linear = clipper(cammand_robot[0,0], -MaxSpeed, MaxSpeed)

      if DEBUG_FLAG:
         print(" Robot Command = ",command_linear, command_angular)
         time.sleep(0.5)

      dx_prev = dx
      # commands = [command_linear, command_angular]

      return command_linear, command_angular

   except e:
      print("Exception ...")

def publish_command(command_linear, command_angular, t):
   global data_remote, command_publisher
   Ctime = round(time.clock(),4)
   data_remote.data = [Ctime,command_linear,command_angular]
   command_publisher.publish(data_remote)
   rospy.loginfo(data_remote)


def trajectory_service(t):
   # print "Waiting for RDS Service"
   try:
      # x, y, phi = pose

      (Trajectory_V, Trajectory_W) = ds_generation(*pose)
      if ~DEBUG_FLAG:
         publish_command(Trajectory_V, Trajectory_W, t)
   except:
        publish_command(0., 0., 0.)
        print ('No Pose Setting [V,W] = [0, 0]')


def main():
   global command_publisher, data_remote, trajectory_xyt
   rospy.init_node('qolo_ds_trajectory')

   pose_sub = rospy.Subscriber("qolo/pose2D", Pose2D, pose_callback, queue_size=1)
   control_pt_sub = rospy.Subscriber("qolo/control_pt", Pose2D, control_pt_callback, queue_size=1)
   command_publisher = rospy.Publisher('qolo/remote_commands',Float32MultiArray, queue_size=1)

   data_remote.layout.dim.append(MultiArrayDimension())
   data_remote.layout.dim[0].label = 'Trajectory Commands [V, W]'
   data_remote.layout.dim[0].size = 3
   data_remote.data = [0]*3

   # end_time = trajectory_xyt[-1][2]
   print('Trajectory time: ',time_limit)
   start_time = time.time()
   while not rospy.is_shutdown():
      current_t = time.time() - start_time
      if current_t <= time_limit:
         trajectory_service(current_t)
      else:
         publish_command(0., 0., 0.)
         print ('End of Trajectory set to --[0 , 0]')
         time.sleep(0.5)
         publish_command(0., 0., 0.)
         time.sleep(0.5)
         break

if __name__ == '__main__':
   main()
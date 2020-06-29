#! /usr/bin/env python
#########  ROS version of Trajectory Tracking with safety ##########
##### Author: Diego F. Paez G. & David Gonon
##### Preliminar version: David Gonon
##### Data: 2020/05/18

import time
import math
import rospy
# from rds_network_ros.srv import * #VelocityCommandCorrectionRDS
import tf
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension 
import numpy as np
from scipy.interpolate import UnivariateSpline
import dynamical_system_representation as ds
#import matplotlib.pyplot as plt

dx_prev = np.array([[0.0], [0.0]])
dx = np.array([[0.0], [0.0]])

DEBUG_FLAG = False
MaxSpeed = 1.5/2 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
MaxAngular = 4.124/4
D_angular = 10
D_linear = 10

ref_vel = 0.8
control_point = 0.9
stop_distance = 0.5
time_limit = 90

Attractor = np.array([[10.0+control_point], [0.0]])

tf_listener = None
command_publisher = None
t_lost_tf = -1.0
previous_command_linear = None
previous_command_angular = None
data_remote = Float32MultiArray()

def get_pose():
   global tf_listener
   (trans, rot) = tf_listener.lookupTransform('/tf_qolo_world', '/tf_qolo', rospy.Time(0))
   rpy = tf.transformations.euler_from_quaternion(rot)
   print ("phi=", rpy[2])
   return (trans[0], trans[1], rpy[2])

def ds_generation(x,y,phi):
   global dx_prev, dx, previous_time, ref_vel
   try:
      # x = 1
      # y = 1
      # phi = 0.79
      t_lost_tf = -1.0
      Ctime = time.clock()
      translation = np.array([[x], [y]])
      Rotation = np.array([
         [np.cos(phi), -np.sin(phi)],
         [np.sin(phi),  np.cos(phi)]])
      p_ref_local = np.array([[control_point], [0.0]])
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

      command_linear = cammand_robot[0,0]

      if cammand_robot[1,0] > MaxAngular:
         command_angular = MaxAngular
      elif cammand_robot[1,0] < -MaxAngular:
         command_angular = -MaxAngular
      else:
         command_angular = cammand_robot[1,0]
      # previous_command_linear = command_linear
      # previous_command_angular = command_angular
      if DEBUG_FLAG:
         print(" Robot Command = ",command_linear, command_angular)
         time.sleep(0.5)
      dx_prev = dx
      # commands = [command_linear, command_angular]

      return command_linear, command_angular

   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print ("Exception during tf lookup ...")


def publish_command(command_linear, command_angular, t):
   global data_remote, command_publisher
   Ctime = round(time.clock(),4)
   data_remote.data = [Ctime,command_linear,command_angular]
   command_publisher.publish(data_remote)
   rospy.loginfo(data_remote)


def trajectory_service(t):
   # print "Waiting for RDS Service"
   try:
      (x, y, phi) = get_pose()

      (Trajectory_V, Trajectory_W) = ds_generation(x,y,phi)
      if ~DEBUG_FLAG:
         publish_command(Trajectory_V, Trajectory_W, t)
   except:
        publish_command(0., 0., 0.)
        print ('No Pose Setting [V,W] = [0, 0]')


def main():
   global tf_listener, command_publisher, data_remote, trajectory_xyt
   rospy.init_node('qolo_ds_trajectory')
   tf_listener = tf.TransformListener()
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
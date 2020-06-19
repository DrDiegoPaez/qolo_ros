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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import numpy as np
from scipy.interpolate import UnivariateSpline
import dynamical_system_representation as ds
#import matplotlib.pyplot as plt

dx_prev = np.array([[0.0], [0.0]])
dx = np.array([[0.0], [0.0]])
qolo_pose = np.array([0. 0. 0.])

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

def create_spline_curve(XYT):
   return [
      UnivariateSpline(XYT[:, 2], XYT[:, 0], k=2, s=0, ext=3),
      UnivariateSpline(XYT[:, 2], XYT[:, 1], k=2, s=0, ext=3)]

def plot_spline_curve(spline_curve, time_vector, data_xyt):
   v_x = spline_curve[0].derivative()
   v_y = spline_curve[1].derivative()
   tau = 4.0
   fig = plt.figure()
   ax = fig.add_subplot(111)
   for t in time_vector:
      tail_x = spline_curve[0](t)
      tail_y = spline_curve[1](t)
      head_x = tail_x + tau*v_x(t)
      head_y = tail_y + tau*v_y(t)
      ax.plot([tail_x, head_x], [tail_y, head_y], 'g')
   ax.plot(spline_curve[0](time_vector), spline_curve[1](time_vector), 'ro-')
   ax.plot(data_xyt[:,0], data_xyt[:,1], 'ko')
   ax.set_aspect('equal')
   ax.set(xlabel='x', ylabel='y')
   plt.show()
   fig, axs = plt.subplots(2)
   axs[0].plot(time_vector, spline_curve[0](time_vector))
   axs[0].set(xlabel='t', ylabel='x')
   axs[1].plot(time_vector, spline_curve[1](time_vector))
   axs[1].set(xlabel='t', ylabel='y')
   plt.show()


   trajectory_spline = create_spline_curve(trajectory_xyt)
   trajectory_spline_derivative = [trajectory_spline[0].derivative(), trajectory_spline[1].derivative()]


def odom_callback(msg):
    global qolo_pose
    
    qolo_pose[0] = (msg.pose.pose.position.x)
    qolo_pose[1] = (msg.pose.pose.position.y)
    qolo_pose[2] = (msg.pose.pose.orientation.z)
    rate.sleep()


def get_pose():
   global tf_listener
   (trans, rot) = tf_listener.lookupTransform('/tf_qolo_world', '/tf_qolo', rospy.Time(0))
   rpy = tf.transformations.euler_from_quaternion(rot)
   print ("phi=", rpy[2])
   return (trans[0], trans[1], rpy[2])

def feedforward_feedback_controller(t):
   global t_lost_tf
   global previous_command_linear
   global previous_command_angular
   global trajectory_spline
   global trajectory_spline_derivative
   
   try:
      (x, y, phi) = get_pose()
      t_lost_tf = -1.0

      R = np.array([
         [np.cos(phi), -np.sin(phi)],
         [np.sin(phi),  np.cos(phi)]])
      translation = np.array([[x], [y]])

      p_ref_local = np.array([[0.0], [0.3]])
      p_ref_global = np.matmul(R, p_ref_local) + translation

      feedforward_velocity = np.array([[trajectory_spline_derivative[0](t)],
         [trajectory_spline_derivative[1](t)]])
      position_setpoint = np.array([[trajectory_spline[0](t)],
         [trajectory_spline[1](t)]])
      feedback_velocity = 0.2*(position_setpoint - p_ref_global)
      v_norm_max = 1.0
      v_norm_actual = np.linalg.norm(feedback_velocity)
      if (v_norm_actual > v_norm_max):
         feedback_velocity = feedback_velocity/v_norm_actual*v_norm_max

      v_command_p_ref_global = feedforward_velocity + feedback_velocity
      # print ("v_command_p_ref_global=", v_command_p_ref_global[0], v_command_p_ref_global[1])
      v_command_p_ref_local = np.matmul(np.transpose(R), v_command_p_ref_global)
      # print ("v_command_p_ref_local=", v_command_p_ref_local[0], v_command_p_ref_local[1])

      J_p_ref_inv = np.array([
         [p_ref_local[0,0]/p_ref_local[1,0], 1.0],
         [-1.0/p_ref_local[1,0], 0.0]])
      command_linear_angular = np.matmul(J_p_ref_inv, v_command_p_ref_local)
      command_linear = command_linear_angular[0]
      command_angular = command_linear_angular[1]
      previous_command_linear = command_linear
      previous_command_angular = command_angular
      return (command_linear, command_angular)
   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print ("Exception during tf lookup ...")
      if t_lost_tf == -1.0:
         t_lost_tf = t
      decay_factor = 1.0 - np.min([2.0, t - t_lost_tf])/2.0
      if type(previous_command_linear) == type(None):
         return (0.0, 0.0)
      return (decay_factor*previous_command_linear,
         decay_factor*previous_command_angular)


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

def publish_qolo_tf(x, y, phi):
   br = tf.TransformBroadcaster()
   br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, phi),
                     rospy.Time.now(),
                     "tf_qolo",
                     "tf_qolo_world")

def trajectory_service(t):
   # print "Waiting for RDS Service"
   global qolo_x
   try:
      # (x, y, phi) = get_pose()
      (x, y, phi) =  qolo_pose[0], qolo_pose[1], qolo_pose[2]
      publish_qolo_tf(x, y, phi)
      (Trajectory_V, Trajectory_W) = ds_generation(x,y,phi)
      if ~DEBUG_FLAG:
         publish_command(Trajectory_V, Trajectory_W, t)
   except:
        publish_command(0., 0., 0.)
        print ('No Pose Setting [V,W] = [0, 0]')


def main():
   global tf_listener, command_publisher, data_remote, trajectory_xyt, qolo_pose
   rospy.init_node('qolo_ds_trajectory')
   tf_listener = tf.TransformListener()
   command_publisher = rospy.Publisher('qolo/remote_commands',Float32MultiArray, queue_size=1)
   odometry_qolo = rospy.Subscriber("/qolo/odom",Odometry,odom_callback, queue_size=1)
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
#! /usr/bin/env python3
#########  ROS version of Trajectory Tracking with safety ##########
##### Author: Diego F. Paez G. & David Gonon
##### Preliminar version: David Gonon
##### Data: 2020/05/18

import time
import math
import rospy
import signal
# from rds_network_ros.srv import * #VelocityCommandCorrectionRDS
import tf
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import numpy as np
from scipy.interpolate import UnivariateSpline
import dynamical_system_representation as ds
from rds_network_ros.srv import *

from metrics_evaluation import RobotMetricsEvaluation

#import matplotlib.pyplot as plt
command_publisher = rospy.Publisher('qolo/twist_cmd', Twist, queue_size=1)
# data_remote = Float32MultiArray()
qolo_twist = Twist()

dx_prev = np.array([[0.0], [0.0]])
dx = np.array([[0.0], [0.0]])
qolo_pose = np.array([0., 0., 0.])
Start_pose = np.array([0., 0., 0.])

DEBUG_FLAG = False
MaxSpeed = 1.5 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
MaxAngular = 4.124
D_angular = 10
D_linear = 10

ref_vel = 0.5
density = 1.0
control_point = 0.9
stop_distance = 0.1
time_limit = 1090

Attractor = np.array([[40.0+control_point], [0.0]])

robot_metrics_eval = RobotMetricsEvaluation(Attractor[0,0], Attractor[1,0])

#tf_listener = tf.TransformListener()
tf_listener = None
t_lost_tf = -1.0
previous_command_linear = None
previous_command_angular = None

#########################################################
############ Setting for the RDS service ################
#########################################################

# y_coordinate_of_reference_point_for_command_limits = 0.5;
# Gain to this point
weight_scaling_of_reference_point_for_command_limits = 0.;
# Some gain for velocity after proximity reaches limits
tau = 1.5;
# Minimal distance to obstacles
delta = 0.05;
# Some reference for controlling the non-holonomic base
control_point_rds = 0.17;

max_linear = MaxSpeed;
min_linear = -MaxSpeed;
absolute_angular_at_min_linear = 0.;
absolute_angular_at_max_linear = 0.;
absolute_angular_at_zero_linear = MaxAngular;
linear_acceleration_limit = 4.0
angular_acceleration_limit = 7.0

Corrected_V = 0.;
Corrected_W = 0.;
cycle = 0.1;

def rds_service(User_V, User_W):
    global cycle, Corrected_V, Corrected_W
    # print "Waiting for RDS Service"
    rospy.wait_for_service('rds_velocity_command_correction')
    try:
        RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

        request = VelocityCommandCorrectionRDSRequest()
        # y_coordinate_of_reference_point_for_command_limits = 0.5;
        # # Gain to this point
        # weight_scaling_of_reference_point_for_command_limits = 0.;
        # # Some gain for velocity after proximity reaches limits
        # tau = 2.;
        # # Minimal distance to obstacles
        # delta = 0.08;
        # clearance_from_axle_of_final_reference_point = 0.15;
        # max_linear = MaxSpeed;
        # min_linear = -MinSpeed;
        # absolute_angular_at_min_linear = 0.;
        # absolute_angular_at_max_linear = 0.;
        # absolute_angular_at_zero_linear = MaxAngular/W_ratio;
        # linear_acceleration_limit = 1.1
        # angular_acceleration_limit = 1.5

        request.nominal_command.linear = User_V;
        request.nominal_command.angular = User_W;
        request.capsule_center_front_y = 0.2;
        request.capsule_center_rear_y = -0.50;
        request.capsule_radius = 0.3;
        
        request.reference_point_y = control_point_rds;

        request.rds_tau = tau;  # Time horizon for velocity obstacles
        request.rds_delta = delta;
        request.vel_lim_linear_min = -0.75;
        request.vel_lim_linear_max = 1.5;
        request.vel_lim_angular_abs_max = 4.0;
        request.vel_linear_at_angular_abs_max = 0.1;
        request.acc_limit_linear_abs_max = 2.0;
        request.acc_limit_angular_abs_max = 7.0;

        # // shall rds consider lrf measurements?
        request.lrf_point_obstacles = True;
        # // for generating constraints due to raw lrf scan points,
        # // shall rds use the VO-based or the alternative (prior) approach?
        request.lrf_alternative_rds = False;
        # // how shall rds choose the base velocity for determining the convex approximate VO
        # // 0 : use zero velocity (ensures that the final halfplane contains the VO, if the VO does not contain the origin)
        # // 1 : use the velocity which rds computed previously
        # // any other integer: use the nominal velocity (from the current nominal command)
        request.vo_tangent_base_command = 0;
        # // shall rds map the base velocity to the tangent point the same way as ORCA for determining the convex approximate VO?
        request.vo_tangent_orca_style = True;

        if cycle==0:
            delta_time = 0.05;
        else:
            delta_time = time.clock() - cycle;

        request.dt = delta_time #0.01 #delta_time

        response = RDS(request)
        Corrected_V = round(response.corrected_command.linear,6)
        Corrected_W = round(response.corrected_command.angular,6)

        cycle = time.clock()
    except:
        Corrected_V = 0.
        Corrected_W = 0.

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
    # rate.sleep()

def pose_callback(msg):
    global qolo_pose, Start_pose
    qolo_pose[0] = (msg.linear.y - Start_pose[0])
    qolo_pose[1] = (msg.linear.x - Start_pose[1])
    qolo_pose[2] = ((msg.angular.z)*np.pi/180.) - Start_pose[2]


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


def publish_command(Vel,Omega,time):
    global command_publisher, qolo_twist
    # data_remote.data = [time,Vel,Omega]
    qolo_twist.linear.x = Vel
    qolo_twist.angular.z = -Omega*180./np.pi

    command_publisher.publish(qolo_twist)
    rospy.loginfo(qolo_twist)


def publish_qolo_tf(x, y, phi):
   br = tf.TransformBroadcaster()
   br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, phi),
                     rospy.Time.now(),
                     "tf_qolo",
                     "tf_qolo_world")

def trajectory_service(t):
   # print "Waiting for RDS Service"
   global qolo_x, Corrected_V, Corrected_W, robot_metrics_eval
   try:
      # (x, y, phi) = get_pose()
      (x, y, phi) =  qolo_pose[0], qolo_pose[1], qolo_pose[2]
      robot_metrics_eval.update(x, y, t)
      publish_qolo_tf(x, y, phi)
      (Trajectory_V, Trajectory_W) = ds_generation(x,y,phi)
      rds_service(Trajectory_V, Trajectory_W)
      (Corrected_V, Corrected_W) = Trajectory_V, Trajectory_W
      if ~DEBUG_FLAG:
         publish_command(Corrected_V, -Corrected_W, t)
   except:
        publish_command(0., 0., 0.)
        print ('No Pose Setting [V,W] = [0, 0]')


def main():
   global tf_listener, qolo_twist, trajectory_xyt, qolo_pose
   rospy.init_node('qolo_simulator_ds', anonymous=True)
   rate = rospy.Rate(100) #  50 hz
   pose_qolo = rospy.Subscriber("/qolo/pose",Twist,pose_callback, queue_size=1)
   #odometry_qolo = rospy.Subscriber("/qolo/odom",Odometry,odom_callback, queue_size=1)
   qolo_twist = Twist()
   # qolo_twist.header = make_header("tf_qolo") # for visualization
   qolo_twist.linear.x = 0
   qolo_twist.linear.y = 0
   qolo_twist.linear.z = 0
   qolo_twist.angular.x = 0
   qolo_twist.angular.y = 0
   qolo_twist.angular.z = 0
   time.sleep(0.5)
   
   # Setting start pose to current one (if wanted)
   # (Start_pose[0], Start_pose[1], Start_pose[2]) = qolo_pose[0], qolo_pose[1], qolo_pose[2]
   # Setting Start pose to simulator initial pose
   (Start_pose[0], Start_pose[1], Start_pose[2]) = -20, 0, (-270)*np.pi/180.
   print('Trajectory time: ',time_limit)
   start_time = time.time()
   print('Start time: ',start_time)
   print('Start pose: ',Start_pose[0],Start_pose[1],Start_pose[2])
   time.sleep(2.0)
   GoalFlag = False
   while not rospy.is_shutdown():
      current_t = time.time() - start_time
      if (current_t <= time_limit) or GoalFlag:
          trajectory_service(current_t)
          dx = Attractor-qolo_pose
          dx_mag = np.sqrt(np.sum(dx**2))
          if dx_mag <= stop_distance:
            GoalFlag=True

          #print('current time: ',current_t)
          #print('current pose: ',qolo_pose[0],qolo_pose[1],qolo_pose[2])
          #time.sleep(0.5)
      else:
         publish_qolo_tf(qolo_pose[0],qolo_pose[1],qolo_pose[2])
         publish_command(0., 0., 0.)
         print ('End of Trajectory set to --[0 , 0]')
         time.sleep(0.5)
         # publish_command(0., 0., 0.)
         # time.sleep(0.5)
         # break
# for interruptions
def exit(signum, frame):
    global robot_metrics_eval
    robot_metrics_eval.print_result("v%f-d%f.txt" % (ref_vel, density))
    # Stop_Thread_Flag = True
    # cleanup_stop_thread()
    print('---> You chose to interrupt')
    quit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      time.sleep(0.1)
      pass
   robot_metrics_eval.print_result("v%f-d%f.txt" % (ref_vel, density))

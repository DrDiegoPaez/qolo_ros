#! /usr/bin/env python3

#########  Qolo Main Code for Shared / Embodied / Remore Control ##########
##### Author: Diego F. Paez G.
##### Collaboration: Vaibhav Gupta
##### Embodied sensing: Chen Yang
##### Data: 2019/10/01

from ADDA import ADDA as converter
import time
import math
import os
from itertools import groupby
import numpy as np
import array as arr
# from scipy.signal import butter, lfilter, freqz
# import matplotlib.pyplot as plt
import heapq
import logging
from logging import handlers
import mraa
import signal
import datetime
import rospy
# import threading

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from rds_network_ros.srv import *
# from builtins import PermissionError

from filters import MultiLowPassFilter
from logger import Logger
from compliance_controller import AdmittanceController

from termcolor import colored

# FLAG for fully manual control (TRUE) or shared control (FALSE)
#Tonado server port
try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    print(colored("Cannot set niceness for the process...", "red"))
    print(colored("Run the script as sudo...", "red"))

K_vel = 0.7
# CONSTANT_VEL_MODE = True
CONSTANT_VEL_MODE = rospy.get_param("/qolo_control/constant_mode", False)
# For testing collision avoidance 
SHARED_MODE = rospy.get_param("/qolo_control/shared_mode", False)
# For testing collision control
COMPLIANCE_MODE = rospy.get_param("/qolo_control/compliance_mode", False)
# For using remote app Joystick
JOYSTICK_MODE = rospy.get_param("/qolo_control/joystick_mode", False)
# For using DS or trajectory tracking
REMOTE_MODE = rospy.get_param("/qolo_control/remote_mode", False)
# For zero output to the wheels
TESTING_MODE = False
DEBUG_MODE = True
TIMING_MODE = True

PORT = 8080
control_type ='embodied'
conv = converter()

# Logger
logger = Logger(rospy.get_param("/qolo_control/log_folder", "csv_logs"))
print(colored("Logging folder is '{}'".format(logger.folder), "yellow"))

# Fast Clipper Function
clipper = lambda x, l, u: l if x < l else u if x > u else x

# coefficient for vmax and wmax(outout curve)
forward_coefficient = 1
left_turning_coefficient = 1
right_turning_coefficient = 1
backward_coefficient = 0.5

# Global Constatns for Communication
# DAC0 --> Left Wheel Velocity
# DAC1 --> Right Wheel Velocity
# DAC2 --> Enable Qolo Motion
THRESHOLD_V = 1500
ZERO_LW = 0 #2750
ZERO_RW = 0 #2650
HIGH_DAC = 5000
MBED_Enable = mraa.Gpio(36)
MBED_Enable.dir(mraa.DIR_OUT)

GEAR = 12.64            # Gear ratio
DISTANCE_CW = 0.548/2   # DISTANCE_CW bettween two wheels
RADIUS = 0.304/2        # Meter

MAX_SPEED = 1.5         # max Qolo speed: 1.51 m/s --> Equivalent to 5.44 km/h
MIN_SPEED = MAX_SPEED*backward_coefficient
MAX_OMEGA = 4.124
W_RATIO = 3.5           # Ratio of the maximum angular speed (232 deg/s)

MAX_MOTOR_V = 1200.0    # max motor speed: 1200 rpm

MAX_WHEEL_VEL = (95.0/60.0)*(2*np.pi)
WHEEL_ACC = (((4.0/60.0)*(2*np.pi))/GEAR)/(1.0/400.0) # Maximum wheel acceleration = 13.2557 rad/s2
LINEAR_ACC = WHEEL_ACC * RADIUS         # Maximum Robot's linear acceleration = 2.0149 m/s2
ANGULAR_ACC = LINEAR_ACC / DISTANCE_CW  # Maximum Robot's angular acceleration = 7.3535 rad/s2 

#########################################################
############ Setting for the RDS service ################
#########################################################
LRF_points_Flag = True
ORCA_Flag = True
# y_coordinate_of_reference_point_for_command_limits = 0.5
# Gain to this point
weight_scaling_of_reference_point_for_command_limits = 0.
# Some gain for velocity after proximity reaches limits
tau = 1.5
# Minimal distance to obstacles
delta = 0.05
# Some reference for controlling the non-holonomic base
control_point = 0.2

max_linear = MAX_SPEED
min_linear = -MIN_SPEED
absolute_angular_at_min_linear = 0.
absolute_angular_at_max_linear = 0.
absolute_angular_at_zero_linear = MAX_OMEGA/W_RATIO
linear_acceleration_limit = 1.5
angular_acceleration_limit = 4.5

#########################################################
############ Setting for Compliant Control ##############
#########################################################
compliant_V =0.
compliant_W =0.

compliance_control = None

# Global Variables for Compliant mode
offset_ft_data = np.zeros((6,))
raw_ft_data  = np.zeros((6,))
filtered_ft_data  = np.zeros((6,))
ft_data =  np.zeros((6,))
svr_data =  np.zeros((3,))
initialising_ft=True
init_ft_data = {
    'Fx': [],
    'Fy': [],
    'Fz': [],
    'Mx': [],
    'My': [],
    'Mz': [],
    }
bumper_loc = np.zeros((3,))

# Prediction Models
bumperModel = None
lp_filter = None

feasible = 0
Output_V = 0.
Output_W = 0.
Corrected_V = 0.
Corrected_W = 0.
Remote_V = 0.
Remote_W = 0.
FlagRemote = False
last_msg = 0.
time_msg = 0.
Count_msg_lost = 0

last_v = 0.
last_w = 0.
cycle=0.

Command_V = 0.
Command_W = 0.
Comand_DAC0 = 0
Comand_DAC1 = 0
Send_DAC0 = 0.
Send_DAC1 = 0. 
Send_DAC2 = 0.
Send_DAC3 = 0.
# rpm_L = 0
# rpm_R = 0

# Global variables for logging
DA_time = 0.
RDS_time = 0.
Compute_time = 0. 
FSR_time = 0.
Compliance_time = 0.
User_V = 0.
User_W = 0.
Out_CP = 0.
Out_F = 0.
t1=0.
t2=0.

counter1 = 0
number = 100

# real zero point of each sensor
a_zero, b_zero, c_zero, d_zero, e_zero, f_zero, g_zero, h_zero = 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11

# FsrZero = arr.array('d',[200.1 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11 200.1])
# FsrZero = np.array([100.1, 305.17, 264.7, 441.57, 336.46, 205.11, 150.57, 160.46, 150.11, 200.1])
# FsrZero = np.array([304.5, 298.99, 202.69, 405.66, 294.8, 296.8, 334.01, 282.98, 250.73, 208.32])
# Calibration Diego
# FsrZero = np.array([304.5, 298.99, 268.69, 441.66, 416.8, 305.8, 334.01, 202.98, 250.73, 220.32])
FsrZero = np.array([433.0, 1227.0, 803.0, 314.0, 906.0, 930.0, 1047.0, 195.0, 237.0, 159.0])
# default value for pre-configuration
# k1, k2, k3, k4, k5, k6, k7, k8 =    0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 0.57, 0.63 # 2.48, 0.91, 1.59, 1.75, 1.46
# FsrK = np.array([0., 0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 1.04, 0.7, 0.])
FsrK = np.array([0.0, 1.03, 1.07, 1.0, 1.0, 1.0, 1.0, 1.2, 1.35, 0.0])
# Vector input for all sensor data
# Xin = np.zeros((10))
Xin = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
Xin_temp = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
ComError = 0
RemoteE = 0

# # coefficient for calculate center of pressure: ox
Rcenter = np.array([0., -2.5, -1.875, -1.25, -0.625, 0.625, 1.25, 1.875, 2.5, 0.])

# classification point for center of pressure ox(calibration needed)
pl2, pl1, pr1, pr2 = -1.5, -0.7, 0.7, 1.5
# B-2, B-1, B1, B2
# -1.72, 0.075, 1.455, 1.98 
# -2.42, 0.67, 1.27, 1.82  
# -0.97, -0.2, 0.2, 1.17

AA = []
BB = []
CC = []
DD = []
EE = []
FF = []
GG = []
HH = []
OX = []

level_relations = {
        # 'debug':logging.DEBUG,
        'info':logging.INFO,
        # 'warning':logging.WARNING,
        # 'error':logging.ERROR,
        # 'crit':logging.CRITICAL
    }

# for interruption
def exit(signum, frame):
    # MBED_Enable = mraa.Gpio(36) #11 17
    # MBED_Enable.dir(mraa.DIR_OUT)
    MBED_Enable.write(0)
    conv.SetChannel(2, 0)
    conv.SetChannel(3, 0)
    conv.SetChannel(0, 0)
    conv.SetChannel(1, 0)

    print(colored("===== You chose to interrupt =====", "red"))
    quit()


def callback_remote(data):
    global Remote_V, Remote_W, FlagRemote, time_msg
    
    if time_msg != 0:
        Remote_V = round(data.data[1],8)
        Remote_W =  round(data.data[2],8)
        FlagRemote = True
        time_msg = data.data[0]
    time_msg = data.data[0]

## Compliant control functions
def ft_sensor_callback(data):
    global raw_ft_data, filtered_ft_data, init_ft_data, initialising_ft, ft_data
    _x = data.wrench
    raw_ft_data = np.array([
        _x.force.x,
        _x.force.y,
        _x.force.z,
        _x.torque.x,
        _x.torque.y,
        _x.torque.z,
    ])

    if initialising_ft:
        init_ft_data['Fx'] = np.append(init_ft_data['Fx'], _x.force.x)
        init_ft_data['Fy'] = np.append(init_ft_data['Fy'], _x.force.y)
        init_ft_data['Fz'] = np.append(init_ft_data['Fz'], _x.force.z)
        init_ft_data['Mx'] = np.append(init_ft_data['Mx'], _x.torque.x)
        init_ft_data['My'] = np.append(init_ft_data['My'], _x.torque.y)
        init_ft_data['Mz'] = np.append(init_ft_data['Mz'], _x.torque.z)
    else:
        ft_data = (raw_ft_data - offset_ft_data)
        compliance_control.next_ftdata(ft_data)

# read data from ADDA board
def read_FSR():
    global Xin_temp, RemoteE, ComError
    # Checking emergency inputs
    # RemoteE = conv.ReadChannel(7)
    # ComError = conv.ReadChannel(6)
     # ADC_Value[0]*5.0/0x7fffff
    Xin_temp[0] = round(conv.ReadChannel(5),4)
    Xin_temp[1] = round(conv.ReadChannel(15),4)
    Xin_temp[2] = round(conv.ReadChannel(14),4)
    Xin_temp[3] = round(conv.ReadChannel(13),4)
    Xin_temp[4] = round(conv.ReadChannel(12),4)
    Xin_temp[5] = round(conv.ReadChannel(11),4)
    Xin_temp[6] = round(conv.ReadChannel(10),4)
    Xin_temp[7] = round(conv.ReadChannel(9),4)
    Xin_temp[8] = round(conv.ReadChannel(8),4)
    Xin_temp[9] = round(conv.ReadChannel(4),4)
    # return Xin_temp

# execution command to DAC board based on the output curve
def FSR_execution():
    global pl2, pl1, pr1, pr2
    global Command_V, Command_W, Comand_DAC0, Comand_DAC1
    global Xin, FsrZero, FsrK, Out_CP
    
    a = Xin[1] #--> Sensor 2
    b = Xin[2]
    c = Xin[3]
    d = Xin[4]
    e = Xin[5]
    f = Xin[6]
    g = Xin[7]
    h = Xin[8] # --> Sensor 9
    ox = Out_CP

    treshold  = 700 # avoid unstoppable and undistinguishable
    
    forward, backward, left_angle_for, left_angle_turn, right_angle_for, right_angle_turn, left_around, right_around = FSR_output(a, b, c, d, e, f, g, h, ox)

    if ox <= pr1 and ox >= pl1 and d >= treshold and e >= treshold:
        Command_V = forward
        Command_W = 2500
        # continue
    # turn an angle
    elif (ox <= pl1 and ox >= pl2) and h <= treshold and (c >= treshold or b >= treshold):
        Command_V = left_angle_for
        Command_W = left_angle_turn
        # continue
    elif (ox >= pr1 and ox <= pr2) and a <= treshold and (f >= treshold or g >= treshold):
        Command_V = right_angle_for
        Command_W = right_angle_turn
        # continue
    # turn around
    elif ox <= pl2 and h <= treshold and (a >= treshold or b >= treshold):
        Command_V = 2500
        Command_W = left_around
        # continue
    elif ox >= pr2 and a<=treshold and (g >= treshold or h >= treshold):
        Command_V = 2500
        Command_W = right_around
        
    # backward
    elif a >= treshold and h >= treshold and d < 300 and e < 300:
        Command_V = backward
        Command_W = 2500
        
    else:
        Command_V = 2500
        Command_W = 2500


def enable_mbed():
    MBED_Enable.write(0)
    time.sleep(1)
    MBED_Enable.write(1)
    print("Initiating MBED")
    time.sleep(2)

    StartFlag = 1
    conv.SetChannel(2, HIGH_DAC)
    conv.SetChannel(3, HIGH_DAC)
    conv.SetChannel(0, ZERO_LW)
    conv.SetChannel(1, ZERO_RW)

    ComError = conv.ReadChannel(6)    
    while StartFlag:
        print("Waiting MBED_Enable")
        time.sleep(0.5)
        ComError = conv.ReadChannel(6)
        print("ComError = ",ComError)
        if ComError>THRESHOLD_V:
            StartFlag=0

    time.sleep(1)

def transformTo_Lowevel(Desired_V, Desired_W):
    # A function to transform linear and angular velocities to output commands
    global DISTANCE_CW, RADIUS, User_V, User_W, MAX_SPEED, GEAR, MAX_MOTOR_V
    # Using the desired velocity (linearn adn angular) --> transform to motor speed

    wheel_L = Desired_V - (DISTANCE_CW * Desired_W)    # Output in [m/s]
    wheel_R = Desired_V + (DISTANCE_CW * Desired_W)    # Output in [m/s]
    # print ('Wheels Vel =', wheel_L, wheel_R)

    # Transforming from rad/s to [RPM]
    motor_l = (wheel_L/RADIUS) * GEAR *(30/np.pi)
    motor_r = (wheel_R/RADIUS) * GEAR *(30/np.pi)
    # print ('Motor Vel =', motor_l, motor_r)    

    # Transforming velocities to mV [0-5000]
    Command_L = round( (ZERO_LW + HIGH_DAC*motor_l / MAX_MOTOR_V), 6)
    Command_R = round( (ZERO_RW + HIGH_DAC*motor_r / MAX_MOTOR_V), 6)

    return Command_L, Command_R


def write_DA(Write_DAC0,Write_DAC1):
    global Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3
    # ADC Board output in mV [0-5000]

    # Inverting for sending absolute values of velocity
    if Write_DAC0 < 0:
        Write_DAC0 = -Write_DAC0
        Send_DAC2 = 2500
    else:
        Send_DAC2 = 5000

    # Inverting for sending absolute values of velocity
    if Write_DAC1 < 0:
        Write_DAC1 = -Write_DAC1
        Send_DAC3 = 2500
    else:
        Send_DAC3 = 5000

    # Capping the maximum values
    if Write_DAC0 > 4900:
        Send_DAC0 = 4900
    else:
        Send_DAC0 = Write_DAC0

    if Write_DAC1 > 4900:
        Send_DAC1 = 4900
    else:
        Send_DAC1 = Write_DAC1

    if TESTING_MODE:
        Send_DAC0 = ZERO_LW
        Send_DAC1 = ZERO_RW
        Send_DAC2 = 0
        Send_DAC3 = 0
    # print('Output: ', Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3)

    conv.SetChannel(2, Send_DAC2)
    conv.SetChannel(3, Send_DAC3)
    conv.SetChannel(0, Send_DAC0)
    conv.SetChannel(1, Send_DAC1)

# output curve: Linear/Angular Velocity-Pressure Center
def FSR_output(a, b, c, d, e, f, g, h, ox):
    global forward_coefficient, left_turning_coefficient, right_turning_coefficient, backward_coefficient
    #sending value setting
    # static_value = 20
    dynamic_value = max(a,b,c,d,e,f,g,h)
    # drive = dynamic_value - static_value
    drive = dynamic_value

    forward = 2500 + forward_coefficient * drive
    if ox > 0 or ox < 0:
        left_around = 2500 + left_turning_coefficient * drive * ox / abs(ox)
    else:
        left_around = 2500
    if ox > 0 or ox < 0:
        right_around = 2500 + right_turning_coefficient * drive * ox / abs(ox)
    else:
        right_around = 2500

    global pl2, pl1, pr1, pr2

    wl = math.pi / (pl2 - pl1) # w for smooth fucntion: sin(wx)
    fai_l_for = math.pi / 2 - wl * pl1
    fai_l_turn = math.pi / 2 - wl * pl2

    wr = math.pi / (pr2 - pr1) # w for smooth fucntion: sin(wx)
    fai_r_for = math.pi / 2 - wr * pr1
    fai_r_turn = math.pi / 2 - wr * pr2

    left_angle_for = 2500 + forward_coefficient * drive / 2 + forward_coefficient * drive / 2 * math.sin(wl * ox + fai_l_for)
    left_angle_turn = 2500 - left_turning_coefficient * drive / 2 - left_turning_coefficient * drive / 2 * math.sin(wl * ox + fai_l_turn)
    right_angle_for = 2500 + forward_coefficient * drive / 2 + forward_coefficient * drive / 2 * math.sin(wr * ox + fai_r_for)
    right_angle_turn = 2500 + right_turning_coefficient * drive / 2 + right_turning_coefficient * drive / 2 * math.sin(wr * ox + fai_r_turn)

    backward = 2500 - backward_coefficient * drive

    # threshold value for keep safety(beyond this value, the joystick will report an error)
    if forward >= 4800:
        forward = 4800
    if left_angle_for >= 4800:
        left_angle_for = 4800
    if right_angle_for >= 4800:
        right_angle_for = 4800
    if right_around >= 4800:
        right_around = 4800
    if left_around <= 300:
        left_around = 300
    if right_angle_turn >= 4800:
        right_angle_turn = 4800
    if left_angle_turn <= 300:
        left_angle_turn = 300
    if backward <= 800:
        backward = 800

    return forward, backward, left_angle_for, left_angle_turn, right_angle_for, right_angle_turn, left_around, right_around

def rds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible, Corrected_V, Corrected_W
    # print "Waiting for RDS Service"
    rospy.wait_for_service('rds_velocity_command_correction')
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()
    # y_coordinate_of_reference_point_for_command_limits = 0.5
    # # Gain to this point
    # weight_scaling_of_reference_point_for_command_limits = 0.
    # # Some gain for velocity after proximity reaches limits
    # tau = 2.
    # # Minimal distance to obstacles
    # delta = 0.08
    # clearance_from_axle_of_final_reference_point = 0.15
    # max_linear = MAX_SPEED
    # min_linear = -MIN_SPEED
    # absolute_angular_at_min_linear = 0.
    # absolute_angular_at_max_linear = 0.
    # absolute_angular_at_zero_linear = MAX_OMEGA/W_RATIO
    # linear_acceleration_limit = 1.1
    # angular_acceleration_limit = 1.5

    request.nominal_command.linear = User_V
    request.nominal_command.angular = User_W
    request.capsule_center_front_y = 0.2 # Actual: 0.051
    request.capsule_center_rear_y = -0.50
    request.capsule_radius = 0.45
    
    request.reference_point_y = control_point

    request.rds_tau = tau  # Time horizon for velocity obstacles
    request.rds_delta = delta
    request.vel_lim_linear_min = min_linear
    request.vel_lim_linear_max = max_linear
    request.vel_lim_angular_abs_max = absolute_angular_at_zero_linear
    request.vel_linear_at_angular_abs_max = 0.2
    request.acc_limit_linear_abs_max = linear_acceleration_limit
    request.acc_limit_angular_abs_max = angular_acceleration_limit

    # // shall rds consider lrf measurements?
    request.lrf_point_obstacles = LRF_points_Flag
    # Swtching ORCA and RDS
    request.ORCA_implementation = ORCA_Flag 

    # // for generating constraints due to raw lrf scan points,
    # // shall rds use the VO-based or the alternative (prior) approach?
    # request.lrf_alternative_rds = False
    # // how shall rds choose the base velocity for determining the convex approximate VO
    # // 0 : use zero velocity (ensures that the final halfplane contains the VO, if the VO does not contain the origin)
    # // 1 : use the velocity which rds computed previously
    # // any other integer: use the nominal velocity (from the current nominal command)
    # request.vo_tangent_base_command = 0
    # // shall rds map the base velocity to the tangent point the same way as ORCA for determining the convex approximate VO?
    # request.vo_tangent_orca_style = True
    # // shall rds work with bounding circles or find per object the closest incircle in the capsule?
    # // any integer n > 2 : use n bounding circles
    # // any integer <= 2 : use local capsule incircles
    # request.bounding_circles = 2

    if cycle==0:
        delta_time = 0.005
    else:
        delta_time = time.clock() - cycle

    request.dt = 0.01 #delta_time
    # print('call time',delta_time)
    response = RDS(request)
    Corrected_V = round(response.corrected_command.linear,6)
    Corrected_W = round(response.corrected_command.angular,6)

    cycle = time.clock()
    # except:
    #     Corrected_V = 0.
    #     Corrected_W = 0.

def mds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible, Corrected_V, Corrected_W, control_point
    # print "Waiting for MDS Service"

    rospy.wait_for_service('rds_velocity_command_correction')
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()

    request.nominal_command.linear = User_V
    request.nominal_command.angular = User_W
    request.capsule_center_front_y = 0.05
    request.capsule_center_rear_y = -0.5
    request.capsule_radius = 0.45
    request.reference_point_y = control_point
    request.rds_tau = 1.5
    request.rds_delta = 0.05
    request.vel_lim_linear_min = 0.5
    request.vel_lim_linear_max = 1.5
    request.vel_lim_angular_abs_max = 1.0
    request.vel_linear_at_angular_abs_max = 0.2
    request.acc_limit_linear_abs_max = 0.5
    request.acc_limit_angular_abs_max = 0.5
    if cycle==0:
        delta_time = 0.005
    else:
        delta_time = time.clock() - cycle
    request.dt = delta_time

    response = RDS(request)
    Corrected_V = round(response.corrected_command.linear,6)
    Corrected_W = round(response.corrected_command.angular,6)

    # last_v = Output_V
    # last_w = Output_W
    cycle = time.clock()
# print cycle 
# except:
    # Output_V = User_V
    # Output_W = User_W
# print "RDS Service failed"


def joystick_control():
    try:
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(PORT)
        main_loop = tornado.ioloop.IOLoop.instance()

        print ("Tornado Server started")
        main_loop.start()

    except:
        print ("Exception triggered - Tornado Server stopped.")
        # GPIO.cleanup()


def control():
    global A1, B1, C1, D1, E1, F1, G1, H1
    # global r1, r2, r3, r4, r5, r6, r7, r8
    global Rcenter, Count_msg_lost, time_msg, last_msg
    global Command_V, Command_W, Comand_DAC0, Comand_DAC1, User_V, User_W, Output_V, Output_W, last_w, last_v, Corrected_V, Corrected_W
    global counter1, compliant_V, compliant_W, raw_ft_data, ft_data, svr_data, offset_ft_data, compliance_control
    global DA_time, RDS_time, Compute_time, FSR_time, Compliance_time
    
    # Replace with a node subsription
    global Xin, Xin_temp, FsrZero, FsrK, Out_CP
    if TIMING_MODE:
        t1 = time.clock()
        t_start = t1

    if JOYSTICK_MODE or REMOTE_MODE:
        if time_msg == last_msg:
            Count_msg_lost=Count_msg_lost+1
        else:
            last_msg = time_msg
            Count_msg_lost = 0
        
        if Count_msg_lost <10:
            User_V = Remote_V
            User_W = Remote_W
        else:
            User_V = 0.
            User_W = 0.
    elif CONSTANT_VEL_MODE:
        User_V = K_vel
        User_W = 0.
    else:
        read_FSR()
        # FSR Inputs calibration: 
        Xin = FsrK * (Xin_temp - FsrZero)     # Values in [mV]
        for i in range (0,10):
            if Xin[i] < 0.:
                Xin[i] = 0.
        # Calculating the Center of pressure
        ox = np.sum(Rcenter*Xin) / (Xin[1] + Xin[2] + Xin[3] + Xin[4] + Xin[5] + Xin[6] + Xin[7] + Xin[8])
        if math.isnan(ox):
            ox = 0.
        Out_CP = round(ox, 6)
        FSR_execution()  # Runs the user input with Out_CP and returns Command_V and Command_W --> in 0-5k scale
        motor_v = 2*MAX_MOTOR_V*Command_V/5000 - MAX_MOTOR_V            # In [RPM]
        motor_w = (2*MAX_MOTOR_V/(DISTANCE_CW)*Command_W/5000 - MAX_MOTOR_V/(DISTANCE_CW)) / W_RATIO # In [RPM]
        User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),6)
        User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),6)

    if TIMING_MODE:
        FSR_time = round((time.clock() - t1),6)
        t1 = time.clock()

    if SHARED_MODE:
        rds_service()
        # Corrected_V = User_V
        # Corrected_W = User_W
    else:
        Corrected_V = User_V
        Corrected_W = User_W

    if TIMING_MODE:
        RDS_time = round((time.clock() - t1),6)
        t1 = time.clock()

    if COMPLIANCE_MODE:
        (compliant_V, compliant_W) = compliance_control.step(
            compliant_V, compliant_W,
            Corrected_V, Corrected_W,
            svr_data
        )
        Output_V = round(compliant_V,6)
        Output_W = round(compliant_W,6)
    else:
        Output_V = Corrected_V
        Output_W = Corrected_W

    if TIMING_MODE:
        Compliance_time = round((time.clock() - t1),6)
        t1 = time.clock()

    if math.isnan(Output_V):
        Output_V = 0.
    if math.isnan(Output_W):
        Output_W = 0.

    Output_V = clipper(Output_V, -MIN_SPEED, MAX_SPEED)
    Output_W = clipper(Output_W, -MAX_OMEGA, MAX_OMEGA)

    last_v = Output_V
    last_w = Output_W

    Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Output_V, Output_W)
    write_DA(Comand_DAC0, Comand_DAC1)
    if TIMING_MODE:
        DA_time = round((time.clock() - t1),6)
        Compute_time = round((time.clock() - t_start),6)
    # print ('FSR_read: %s, FSR_read: %s, FSR_read: %s, FSR_read: %s,')

    counter1 += 1  # for estiamting frequency


def make_header(frame_name):
        header = Header()
        header.frame_id = frame_name
        header.stamp = rospy.get_rostime()
        return header


def control_node():
    global Comand_DAC0, Comand_DAC1, Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3, Xin
    global RemoteE, ComError
    global DA_time, RDS_time, Compute_time, FSR_time, Compliance_time, extra_time,last_msg, time_msg
    global compliant_V, compliant_W, offset_ft_data, bumperModel, lp_filter, compliance_control, initialising_ft
    prevT = 0
    FlagEmergency=False
    # Call the calibration File
    # load_calibration()
    if COMPLIANCE_MODE:
        compliance_control = AdmittanceController(
            v_max=MAX_SPEED,
            omega_max=(MAX_OMEGA/W_RATIO),
            bumper_l=0.2425,
            bumper_R=0.33,
            Ts=1.0/100,
            Damping_gain=0.1,
            robot_mass=5,
            collision_F_max=45,
            activation_F=15,
            logger=logger
        )
    
    logger.init_topic("corr_velocity", "compliance", ["t", "v_user", "omega_user", "v_OA", "omega_OA", "v_compliance", "omega_compliance", "v_output", "omega_output"])
    if COMPLIANCE_MODE:
        logger.init_topic("raw", "compliance", ["t", "Fx", "Fy", "Fz", "Mx", "My", "Mz"])
    if TIMING_MODE:
        logger.init_topic("timings", "compliance", ["t", "DA_time", "RDS_time", "Compute_time", "FSR_time", "Compliance_time", "Cycle_time"])

    ########### Starting Communication and MBED Board ###########
    ComError = conv.ReadChannel(6)
    if ComError<=THRESHOLD_V:
        enable_mbed()

    ########### Starting ROS Publishers ###########
    pub_emg = rospy.Publisher('qolo/emergency', Bool, queue_size=1)

    pub_twist = rospy.Publisher('qolo/twist', TwistStamped, queue_size=1)
    qolo_twist = TwistStamped()

    # pub_vel = rospy.Publisher('qolo/velocity', Float32MultiArray, queue_size=1)
    # dat_vel = Float32MultiArray()
    # dat_vel.layout.dim.append(MultiArrayDimension())
    # dat_vel.layout.dim[0].label = 'Velocities: last message, Input[2], Output[2]'
    # dat_vel.layout.dim[0].size = 5
    # dat_vel.data = [0]*4

    if DEBUG_MODE:
        pub_cor_vel = rospy.Publisher('qolo/corrected_velocity', Float32MultiArray, queue_size=1)
        dat_cor_vel = Float32MultiArray()
        dat_cor_vel.layout.dim.append(MultiArrayDimension())
        dat_cor_vel.layout.dim[0].label = 'Velocities: User[2] Corrected_OA[2] Corrected_Compliance[2] Output[2] RDS_dT'
        dat_cor_vel.layout.dim[0].size = 9
        dat_cor_vel.data = [0]*9

        pub_wheels = rospy.Publisher('qolo/wheels', Float32MultiArray, queue_size=1)
        dat_wheels = Float32MultiArray()
        dat_wheels.layout.dim.append(MultiArrayDimension())
        dat_wheels.layout.dim[0].label = 'Wheels Output'
        dat_wheels.layout.dim[0].size = 4
        dat_wheels.data = [0]*4

        pub_user = rospy.Publisher('qolo/user_input', Float32MultiArray, queue_size=1)
        dat_user = Float32MultiArray()
        dat_user.layout.dim.append(MultiArrayDimension())
        dat_user.layout.dim[0].label = 'FSR_read'
        dat_user.layout.dim[0].size = 11
        dat_user.data = [0]*11
        
        if COMPLIANCE_MODE:
            pub_compliance_svr = rospy.Publisher('qolo/compliance/svr', WrenchStamped, queue_size=1)
            dat_compliance_svr = WrenchStamped()

            pub_compliance_bumper_loc = rospy.Publisher('qolo/compliance/bumper_loc', Float32MultiArray, queue_size=1)
            dat_compliance_bumper_loc = Float32MultiArray()
            dat_compliance_bumper_loc.layout.dim.append(MultiArrayDimension())
            dat_compliance_bumper_loc.layout.dim[0].label = 'F_mag bumper_theta bumper_h'
            dat_compliance_bumper_loc.layout.dim[0].size = 3
            dat_compliance_bumper_loc.data = [0]*3
    
    ########### Starting ROS Node ###########
    rospy.init_node('qolo_control', anonymous=True)
    rate = rospy.Rate(100) #  100 [Hz]

    if COMPLIANCE_MODE:
        ftsub = rospy.Subscriber("/rokubi_node_front/ft_sensor_measurements",WrenchStamped,ft_sensor_callback, queue_size=1)
        
        initialising_ft = True
        rospy.loginfo('Waiting for FT Sensor Offset: 5 sec')
        # while (time.time() - start_time > 4): # 4 s for initialising ft sensor
        time.sleep(5)
        initialising_ft = False
        offset_ft_data = np.array([
            np.mean(init_ft_data['Fx']),
            np.mean(init_ft_data['Fy']),
            np.mean(init_ft_data['Fz']),
            np.mean(init_ft_data['Mx']),
            np.mean(init_ft_data['My']),
            np.mean(init_ft_data['Mz']),
        ])
        rospy.loginfo('Starting Compliant Mode')
    else:
        print('Starting WITHOUT FT Sensing')
    
    if JOYSTICK_MODE:
        sub_remote = rospy.Subscriber("qolo/remote_commands", Float32MultiArray, callback_remote, queue_size=1)
        control_type = 'joystick'
        print('Subscribed to JOYSTICK Mode')
    elif REMOTE_MODE:
        sub_remote = rospy.Subscriber("qolo/remote_commands", Float32MultiArray, callback_remote, queue_size=1)
        control_type = 'remote'
        print('Subscribed to REMOTE Mode')
    else:
        control_type = 'embodied'
    
    if SHARED_MODE:
        print('STARTING SHARED CONTROL MODE')
    else:
        print('STARTING MANUAL MODE')

    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    while not rospy.is_shutdown():
        if TIMING_MODE:
            prevT = time.clock()

        control()   # Function of control for Qolo
        # # Checking emergency inputs
        RemoteE = conv.ReadChannel(7) # THRESHOLD_V - 1 # conv.ReadChannel(7)
        ComError = conv.ReadChannel(6) # THRESHOLD_V + 1 # conv.ReadChannel(6)
        # print('Comerror', ComError)
        if ComError<=THRESHOLD_V:
            enable_mbed()
        if RemoteE >= THRESHOLD_V:
            print('RemoteE', RemoteE)
            FlagEmergency=True
            time_msg=0.
            while FlagEmergency:
                pub_emg.publish(FlagEmergency)
                conv.SetChannel(3, 0)
                conv.SetChannel(2, 0)
                conv.SetChannel(0, ZERO_LW)
                conv.SetChannel(1, ZERO_RW)
                ResetFSR = conv.ReadChannel(5)
                if ResetFSR >= THRESHOLD_V:
                    print('ResetFSR ', ResetFSR)
                    FlagEmergency=False
                    pub_emg.publish(FlagEmergency)
                    enable_mbed()
                    time_msg=0.
                time.sleep(0.1)
        
        qolo_twist.header = make_header("tf_qolo")
        qolo_twist.twist.linear.x = Output_V
        qolo_twist.twist.angular.z = Output_W

        # dat_vel.data = [time_msg, User_V, User_W, Output_V, Output_W]
        
        if DEBUG_MODE:
            dat_cor_vel.data = [User_V, User_W, Corrected_V, Corrected_W, compliant_V, compliant_W, Output_V, Output_W, RDS_time]
            dat_wheels.data = [Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3]
            dat_user.data = [Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9],Out_CP]
            if COMPLIANCE_MODE:
                dat_compliance_svr.header = make_header("tf_ft_front")
                dat_compliance_svr.wrench.force.x = svr_data[0]
                dat_compliance_svr.wrench.force.y = svr_data[1]
                dat_compliance_svr.wrench.torque.z = svr_data[2]

                dat_compliance_bumper_loc.data = bumper_loc

        if TIMING_MODE:
            cycle_T = time.clock() - prevT

        # ----- CSV Logs -----
        logger.log('corr_velocity',
                   User_V, User_W,
                   Corrected_V, Corrected_W,
                   compliant_V, compliant_W,
                   Output_V, Output_W)
        
        if COMPLIANCE_MODE:
            compliance_control.log()
            logger.log('raw', *ft_data)
        
        if TIMING_MODE:
            logger.log('timings', DA_time, RDS_time, Compute_time, FSR_time, Compliance_time, cycle_T)

        # ----- Publish ROS topics -----
        pub_emg.publish(FlagEmergency)
        pub_twist.publish(qolo_twist)

        # pub_vel.publish(dat_vel)

        if DEBUG_MODE:
            pub_cor_vel.publish(dat_cor_vel)
            pub_wheels.publish(dat_wheels)
            pub_user.publish(dat_user)
            if COMPLIANCE_MODE:
                pub_compliance_svr.publish(dat_compliance_svr)
                pub_compliance_bumper_loc.publish(dat_compliance_bumper_loc)

        rate.sleep()

    logger.exit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass#! /usr/bin/env python3

#########  Qolo Main Code for Shared / Embodied / Remore Control ##########
##### Author: Diego F. Paez G.
##### Collaboration: Vaibhav Gupta
##### Embodied sensing: Chen Yang
##### Data: 2019/10/01

from ADDA import ADDA as converter
import time
import math
import os
from itertools import groupby
import numpy as np
import array as arr
# from scipy.signal import butter, lfilter, freqz
# import matplotlib.pyplot as plt
import heapq
import logging
from logging import handlers
import mraa
import signal
import datetime
import rospy
# import threading

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from rds_network_ros.srv import *
# from builtins import PermissionError

from filters import MultiLowPassFilter
from logger import Logger
from compliance_controller import AdmittanceController

from termcolor import colored

# FLAG for fully manual control (TRUE) or shared control (FALSE)
#Tonado server port
try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    print(colored("Cannot set niceness for the process...", "red"))
    print(colored("Run the script as sudo...", "red"))

K_vel = 0.7
# CONSTANT_VEL_MODE = True
CONSTANT_VEL_MODE = rospy.get_param("/qolo_control/constant_mode", False)
# For testing collision avoidance 
SHARED_MODE = rospy.get_param("/qolo_control/shared_mode", False)
# For testing collision control
COMPLIANCE_MODE = rospy.get_param("/qolo_control/compliance_mode", False)
# For using remote app Joystick
JOYSTICK_MODE = rospy.get_param("/qolo_control/joystick_mode", False)
# For using DS or trajectory tracking
REMOTE_MODE = rospy.get_param("/qolo_control/remote_mode", False)
# For zero output to the wheels
TESTING_MODE = False
DEBUG_MODE = True
TIMING_MODE = True

PORT = 8080
control_type ='embodied'
conv = converter()

# Logger
logger = Logger(rospy.get_param("/qolo_control/log_folder", "csv_logs"))
print(colored("Logging folder is '{}'".format(logger.folder), "yellow"))

# Fast Clipper Function
clipper = lambda x, l, u: l if x < l else u if x > u else x

# coefficient for vmax and wmax(outout curve)
forward_coefficient = 1
left_turning_coefficient = 1
right_turning_coefficient = 1
backward_coefficient = 0.5

# Global Constatns for Communication
# DAC0 --> Left Wheel Velocity
# DAC1 --> Right Wheel Velocity
# DAC2 --> Enable Qolo Motion
THRESHOLD_V = 1500
ZERO_LW = 0 #2750
ZERO_RW = 0 #2650
HIGH_DAC = 5000
MBED_Enable = mraa.Gpio(36)
MBED_Enable.dir(mraa.DIR_OUT)

GEAR = 12.64            # Gear ratio
DISTANCE_CW = 0.548/2   # DISTANCE_CW bettween two wheels
RADIUS = 0.304/2        # Meter

MAX_SPEED = 1.5         # max Qolo speed: 1.51 m/s --> Equivalent to 5.44 km/h
MIN_SPEED = MAX_SPEED*backward_coefficient
MAX_OMEGA = 4.124
W_RATIO = 3.5           # Ratio of the maximum angular speed (232 deg/s)

MAX_MOTOR_V = 1200.0    # max motor speed: 1200 rpm

MAX_WHEEL_VEL = (95.0/60.0)*(2*np.pi)
WHEEL_ACC = (((4.0/60.0)*(2*np.pi))/GEAR)/(1.0/400.0) # Maximum wheel acceleration = 13.2557 rad/s2
LINEAR_ACC = WHEEL_ACC * RADIUS         # Maximum Robot's linear acceleration = 2.0149 m/s2
ANGULAR_ACC = LINEAR_ACC / DISTANCE_CW  # Maximum Robot's angular acceleration = 7.3535 rad/s2 

#########################################################
############ Setting for the RDS service ################
#########################################################
LRF_points_Flag = True
ORCA_Flag = True
# y_coordinate_of_reference_point_for_command_limits = 0.5
# Gain to this point
weight_scaling_of_reference_point_for_command_limits = 0.
# Some gain for velocity after proximity reaches limits
tau = 1.5
# Minimal distance to obstacles
delta = 0.05
# Some reference for controlling the non-holonomic base
control_point = 0.2

max_linear = MAX_SPEED
min_linear = -MIN_SPEED
absolute_angular_at_min_linear = 0.
absolute_angular_at_max_linear = 0.
absolute_angular_at_zero_linear = MAX_OMEGA/W_RATIO
linear_acceleration_limit = 1.5
angular_acceleration_limit = 4.5

#########################################################
############ Setting for Compliant Control ##############
#########################################################
compliant_V =0.
compliant_W =0.

compliance_control = None

# Global Variables for Compliant mode
svr_data =  np.zeros((3,))
bumper_loc = np.zeros((3,))

# Prediction Models
bumperModel = None
lp_filter = None

feasible = 0
Output_V = 0.
Output_W = 0.
Corrected_V = 0.
Corrected_W = 0.
Remote_V = 0.
Remote_W = 0.
FlagRemote = False
last_msg = 0.
time_msg = 0.
Count_msg_lost = 0

last_v = 0.
last_w = 0.
cycle=0.

Command_V = 0.
Command_W = 0.
Comand_DAC0 = 0
Comand_DAC1 = 0
Send_DAC0 = 0.
Send_DAC1 = 0. 
Send_DAC2 = 0.
Send_DAC3 = 0.
# rpm_L = 0
# rpm_R = 0

# Global variables for logging
FULL_time = 0.
DA_time = 0.
RDS_time = 0.
Compute_time = 0. 
FSR_time = 0.
Compliance_time = 0.
User_V = 0.
User_W = 0.
Out_CP = 0.
Out_F = 0.
t1=0.
t2=0.

counter1 = 0
number = 100

# real zero point of each sensor
a_zero, b_zero, c_zero, d_zero, e_zero, f_zero, g_zero, h_zero = 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11

# FsrZero = arr.array('d',[200.1 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11 200.1])
# FsrZero = np.array([100.1, 305.17, 264.7, 441.57, 336.46, 205.11, 150.57, 160.46, 150.11, 200.1])
# FsrZero = np.array([304.5, 298.99, 202.69, 405.66, 294.8, 296.8, 334.01, 282.98, 250.73, 208.32])
# Calibration Diego
# FsrZero = np.array([304.5, 298.99, 268.69, 441.66, 416.8, 305.8, 334.01, 202.98, 250.73, 220.32])
FsrZero = np.array([433.0, 1227.0, 803.0, 314.0, 906.0, 930.0, 1047.0, 195.0, 237.0, 159.0])
# default value for pre-configuration
# k1, k2, k3, k4, k5, k6, k7, k8 =    0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 0.57, 0.63 # 2.48, 0.91, 1.59, 1.75, 1.46
# FsrK = np.array([0., 0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 1.04, 0.7, 0.])
FsrK = np.array([0.0, 1.03, 1.07, 1.0, 1.0, 1.0, 1.0, 1.2, 1.35, 0.0])
# Vector input for all sensor data
# Xin = np.zeros((10))
Xin = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
Xin_temp = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
ComError = 0
RemoteE = 0

# # coefficient for calculate center of pressure: ox
Rcenter = np.array([0., -2.5, -1.875, -1.25, -0.625, 0.625, 1.25, 1.875, 2.5, 0.])

# classification point for center of pressure ox(calibration needed)
pl2, pl1, pr1, pr2 = -1.5, -0.7, 0.7, 1.5
# B-2, B-1, B1, B2
# -1.72, 0.075, 1.455, 1.98 
# -2.42, 0.67, 1.27, 1.82  
# -0.97, -0.2, 0.2, 1.17

AA = []
BB = []
CC = []
DD = []
EE = []
FF = []
GG = []
HH = []
OX = []

level_relations = {
        # 'debug':logging.DEBUG,
        'info':logging.INFO,
        # 'warning':logging.WARNING,
        # 'error':logging.ERROR,
        # 'crit':logging.CRITICAL
    }

# for interruption
def exit(signum, frame):
    # MBED_Enable = mraa.Gpio(36) #11 17
    # MBED_Enable.dir(mraa.DIR_OUT)
    MBED_Enable.write(0)
    conv.SetChannel(2, 0)
    conv.SetChannel(3, 0)
    conv.SetChannel(0, 0)
    conv.SetChannel(1, 0)

    print(colored("===== You chose to interrupt =====", "red"))
    quit()


def callback_remote(data):
    global Remote_V, Remote_W, FlagRemote, time_msg
    
    if time_msg != 0:
        Remote_V = round(data.data[1],8)
        Remote_W =  round(data.data[2],8)
        FlagRemote = True
        time_msg = data.data[0]
    time_msg = data.data[0]

## Compliant control functions
def ft_sensor_callback(data):
    global svr_data
    _x = data.wrench
    svr_data = np.array([
        _x.force.x,
        _x.force.y,
        _x.torque.z,
    ])

# read data from ADDA board
def read_FSR():
    global Xin_temp, RemoteE, ComError
    # Checking emergency inputs
    # RemoteE = conv.ReadChannel(7)
    # ComError = conv.ReadChannel(6)
     # ADC_Value[0]*5.0/0x7fffff
    Xin_temp[0] = round(conv.ReadChannel(5),4)
    Xin_temp[1] = round(conv.ReadChannel(15),4)
    Xin_temp[2] = round(conv.ReadChannel(14),4)
    Xin_temp[3] = round(conv.ReadChannel(13),4)
    Xin_temp[4] = round(conv.ReadChannel(12),4)
    Xin_temp[5] = round(conv.ReadChannel(11),4)
    Xin_temp[6] = round(conv.ReadChannel(10),4)
    Xin_temp[7] = round(conv.ReadChannel(9),4)
    Xin_temp[8] = round(conv.ReadChannel(8),4)
    Xin_temp[9] = round(conv.ReadChannel(4),4)
    # return Xin_temp

# execution command to DAC board based on the output curve
def FSR_execution():
    global pl2, pl1, pr1, pr2
    global Command_V, Command_W, Comand_DAC0, Comand_DAC1
    global Xin, FsrZero, FsrK, Out_CP
    
    a = Xin[1] #--> Sensor 2
    b = Xin[2]
    c = Xin[3]
    d = Xin[4]
    e = Xin[5]
    f = Xin[6]
    g = Xin[7]
    h = Xin[8] # --> Sensor 9
    ox = Out_CP

    treshold  = 700 # avoid unstoppable and undistinguishable
    
    forward, backward, left_angle_for, left_angle_turn, right_angle_for, right_angle_turn, left_around, right_around = FSR_output(a, b, c, d, e, f, g, h, ox)

    if ox <= pr1 and ox >= pl1 and d >= treshold and e >= treshold:
        Command_V = forward
        Command_W = 2500
        # continue
    # turn an angle
    elif (ox <= pl1 and ox >= pl2) and h <= treshold and (c >= treshold or b >= treshold):
        Command_V = left_angle_for
        Command_W = left_angle_turn
        # continue
    elif (ox >= pr1 and ox <= pr2) and a <= treshold and (f >= treshold or g >= treshold):
        Command_V = right_angle_for
        Command_W = right_angle_turn
        # continue
    # turn around
    elif ox <= pl2 and h <= treshold and (a >= treshold or b >= treshold):
        Command_V = 2500
        Command_W = left_around
        # continue
    elif ox >= pr2 and a<=treshold and (g >= treshold or h >= treshold):
        Command_V = 2500
        Command_W = right_around
        
    # backward
    elif a >= treshold and h >= treshold and d < 300 and e < 300:
        Command_V = backward
        Command_W = 2500
        
    else:
        Command_V = 2500
        Command_W = 2500


def enable_mbed():
    MBED_Enable.write(0)
    time.sleep(1)
    MBED_Enable.write(1)
    print("Initiating MBED")
    time.sleep(2)

    StartFlag = 1
    conv.SetChannel(2, HIGH_DAC)
    conv.SetChannel(3, HIGH_DAC)
    conv.SetChannel(0, ZERO_LW)
    conv.SetChannel(1, ZERO_RW)

    ComError = conv.ReadChannel(6)    
    while StartFlag:
        print("Waiting MBED_Enable")
        time.sleep(0.5)
        ComError = conv.ReadChannel(6)
        print("ComError = ",ComError)
        if ComError>THRESHOLD_V:
            StartFlag=0

    time.sleep(1)

def transformTo_Lowevel(Desired_V, Desired_W):
    # A function to transform linear and angular velocities to output commands
    global DISTANCE_CW, RADIUS, User_V, User_W, MAX_SPEED, GEAR, MAX_MOTOR_V
    # Using the desired velocity (linearn adn angular) --> transform to motor speed

    wheel_L = Desired_V - (DISTANCE_CW * Desired_W)    # Output in [m/s]
    wheel_R = Desired_V + (DISTANCE_CW * Desired_W)    # Output in [m/s]
    # print ('Wheels Vel =', wheel_L, wheel_R)

    # Transforming from rad/s to [RPM]
    motor_l = (wheel_L/RADIUS) * GEAR *(30/np.pi)
    motor_r = (wheel_R/RADIUS) * GEAR *(30/np.pi)
    # print ('Motor Vel =', motor_l, motor_r)    

    # Transforming velocities to mV [0-5000]
    Command_L = round( (ZERO_LW + HIGH_DAC*motor_l / MAX_MOTOR_V), 6)
    Command_R = round( (ZERO_RW + HIGH_DAC*motor_r / MAX_MOTOR_V), 6)

    return Command_L, Command_R


def write_DA(Write_DAC0,Write_DAC1):
    global Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3
    # ADC Board output in mV [0-5000]

    # Inverting for sending absolute values of velocity
    if Write_DAC0 < 0:
        Write_DAC0 = -Write_DAC0
        Send_DAC2 = 2500
    else:
        Send_DAC2 = 5000

    # Inverting for sending absolute values of velocity
    if Write_DAC1 < 0:
        Write_DAC1 = -Write_DAC1
        Send_DAC3 = 2500
    else:
        Send_DAC3 = 5000

    # Capping the maximum values
    if Write_DAC0 > 4900:
        Send_DAC0 = 4900
    else:
        Send_DAC0 = Write_DAC0

    if Write_DAC1 > 4900:
        Send_DAC1 = 4900
    else:
        Send_DAC1 = Write_DAC1

    if TESTING_MODE:
        Send_DAC0 = ZERO_LW
        Send_DAC1 = ZERO_RW
        Send_DAC2 = 0
        Send_DAC3 = 0
    # print('Output: ', Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3)

    conv.SetChannel(2, Send_DAC2)
    conv.SetChannel(3, Send_DAC3)
    conv.SetChannel(0, Send_DAC0)
    conv.SetChannel(1, Send_DAC1)

# output curve: Linear/Angular Velocity-Pressure Center
def FSR_output(a, b, c, d, e, f, g, h, ox):
    global forward_coefficient, left_turning_coefficient, right_turning_coefficient, backward_coefficient
    #sending value setting
    # static_value = 20
    dynamic_value = max(a,b,c,d,e,f,g,h)
    # drive = dynamic_value - static_value
    drive = dynamic_value

    forward = 2500 + forward_coefficient * drive
    if ox > 0 or ox < 0:
        left_around = 2500 + left_turning_coefficient * drive * ox / abs(ox)
    else:
        left_around = 2500
    if ox > 0 or ox < 0:
        right_around = 2500 + right_turning_coefficient * drive * ox / abs(ox)
    else:
        right_around = 2500

    global pl2, pl1, pr1, pr2

    wl = math.pi / (pl2 - pl1) # w for smooth fucntion: sin(wx)
    fai_l_for = math.pi / 2 - wl * pl1
    fai_l_turn = math.pi / 2 - wl * pl2

    wr = math.pi / (pr2 - pr1) # w for smooth fucntion: sin(wx)
    fai_r_for = math.pi / 2 - wr * pr1
    fai_r_turn = math.pi / 2 - wr * pr2

    left_angle_for = 2500 + forward_coefficient * drive / 2 + forward_coefficient * drive / 2 * math.sin(wl * ox + fai_l_for)
    left_angle_turn = 2500 - left_turning_coefficient * drive / 2 - left_turning_coefficient * drive / 2 * math.sin(wl * ox + fai_l_turn)
    right_angle_for = 2500 + forward_coefficient * drive / 2 + forward_coefficient * drive / 2 * math.sin(wr * ox + fai_r_for)
    right_angle_turn = 2500 + right_turning_coefficient * drive / 2 + right_turning_coefficient * drive / 2 * math.sin(wr * ox + fai_r_turn)

    backward = 2500 - backward_coefficient * drive

    # threshold value for keep safety(beyond this value, the joystick will report an error)
    if forward >= 4800:
        forward = 4800
    if left_angle_for >= 4800:
        left_angle_for = 4800
    if right_angle_for >= 4800:
        right_angle_for = 4800
    if right_around >= 4800:
        right_around = 4800
    if left_around <= 300:
        left_around = 300
    if right_angle_turn >= 4800:
        right_angle_turn = 4800
    if left_angle_turn <= 300:
        left_angle_turn = 300
    if backward <= 800:
        backward = 800

    return forward, backward, left_angle_for, left_angle_turn, right_angle_for, right_angle_turn, left_around, right_around

def rds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible, Corrected_V, Corrected_W
    # print "Waiting for RDS Service"
    rospy.wait_for_service('rds_velocity_command_correction')
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()
    # y_coordinate_of_reference_point_for_command_limits = 0.5
    # # Gain to this point
    # weight_scaling_of_reference_point_for_command_limits = 0.
    # # Some gain for velocity after proximity reaches limits
    # tau = 2.
    # # Minimal distance to obstacles
    # delta = 0.08
    # clearance_from_axle_of_final_reference_point = 0.15
    # max_linear = MAX_SPEED
    # min_linear = -MIN_SPEED
    # absolute_angular_at_min_linear = 0.
    # absolute_angular_at_max_linear = 0.
    # absolute_angular_at_zero_linear = MAX_OMEGA/W_RATIO
    # linear_acceleration_limit = 1.1
    # angular_acceleration_limit = 1.5

    request.nominal_command.linear = User_V
    request.nominal_command.angular = User_W
    request.capsule_center_front_y = 0.2 # Actual: 0.051
    request.capsule_center_rear_y = -0.50
    request.capsule_radius = 0.45
    
    request.reference_point_y = control_point

    request.rds_tau = tau  # Time horizon for velocity obstacles
    request.rds_delta = delta
    request.vel_lim_linear_min = min_linear
    request.vel_lim_linear_max = max_linear
    request.vel_lim_angular_abs_max = absolute_angular_at_zero_linear
    request.vel_linear_at_angular_abs_max = 0.2
    request.acc_limit_linear_abs_max = linear_acceleration_limit
    request.acc_limit_angular_abs_max = angular_acceleration_limit

    # // shall rds consider lrf measurements?
    request.lrf_point_obstacles = LRF_points_Flag
    # Swtching ORCA and RDS
    request.ORCA_implementation = ORCA_Flag 

    # // for generating constraints due to raw lrf scan points,
    # // shall rds use the VO-based or the alternative (prior) approach?
    # request.lrf_alternative_rds = False
    # // how shall rds choose the base velocity for determining the convex approximate VO
    # // 0 : use zero velocity (ensures that the final halfplane contains the VO, if the VO does not contain the origin)
    # // 1 : use the velocity which rds computed previously
    # // any other integer: use the nominal velocity (from the current nominal command)
    # request.vo_tangent_base_command = 0
    # // shall rds map the base velocity to the tangent point the same way as ORCA for determining the convex approximate VO?
    # request.vo_tangent_orca_style = True
    # // shall rds work with bounding circles or find per object the closest incircle in the capsule?
    # // any integer n > 2 : use n bounding circles
    # // any integer <= 2 : use local capsule incircles
    # request.bounding_circles = 2

    if cycle==0:
        delta_time = 0.005
    else:
        delta_time = time.clock() - cycle

    request.dt = 0.01 #delta_time
    # print('call time',delta_time)
    response = RDS(request)
    Corrected_V = round(response.corrected_command.linear,6)
    Corrected_W = round(response.corrected_command.angular,6)

    cycle = time.clock()
    # except:
    #     Corrected_V = 0.
    #     Corrected_W = 0.

def mds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible, Corrected_V, Corrected_W, control_point
    # print "Waiting for MDS Service"

    rospy.wait_for_service('rds_velocity_command_correction')
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()

    request.nominal_command.linear = User_V
    request.nominal_command.angular = User_W
    request.capsule_center_front_y = 0.05
    request.capsule_center_rear_y = -0.5
    request.capsule_radius = 0.45
    request.reference_point_y = control_point
    request.rds_tau = 1.5
    request.rds_delta = 0.05
    request.vel_lim_linear_min = 0.5
    request.vel_lim_linear_max = 1.5
    request.vel_lim_angular_abs_max = 1.0
    request.vel_linear_at_angular_abs_max = 0.2
    request.acc_limit_linear_abs_max = 0.5
    request.acc_limit_angular_abs_max = 0.5
    if cycle==0:
        delta_time = 0.005
    else:
        delta_time = time.clock() - cycle
    request.dt = delta_time

    response = RDS(request)
    Corrected_V = round(response.corrected_command.linear,6)
    Corrected_W = round(response.corrected_command.angular,6)

    # last_v = Output_V
    # last_w = Output_W
    cycle = time.clock()
# print cycle 
# except:
    # Output_V = User_V
    # Output_W = User_W
# print "RDS Service failed"


def joystick_control():
    try:
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(PORT)
        main_loop = tornado.ioloop.IOLoop.instance()

        print ("Tornado Server started")
        main_loop.start()

    except:
        print ("Exception triggered - Tornado Server stopped.")
        # GPIO.cleanup()


def control():
    global A1, B1, C1, D1, E1, F1, G1, H1
    # global r1, r2, r3, r4, r5, r6, r7, r8
    global Rcenter, Count_msg_lost, time_msg, last_msg
    global Command_V, Command_W, Comand_DAC0, Comand_DAC1, User_V, User_W, Output_V, Output_W, last_w, last_v, Corrected_V, Corrected_W
    global counter1, compliant_V, compliant_W, raw_ft_data, ft_data, svr_data, offset_ft_data, compliance_control
    global DA_time, RDS_time, Compute_time, FSR_time, Compliance_time
    
    # Replace with a node subsription
    global Xin, Xin_temp, FsrZero, FsrK, Out_CP
    if TIMING_MODE:
        t1 = time.clock()
        t_start = t1

    if JOYSTICK_MODE or REMOTE_MODE:
        if time_msg == last_msg:
            Count_msg_lost=Count_msg_lost+1
        else:
            last_msg = time_msg
            Count_msg_lost = 0
        
        if Count_msg_lost <10:
            User_V = Remote_V
            User_W = Remote_W
        else:
            User_V = 0.
            User_W = 0.
    elif CONSTANT_VEL_MODE:
        User_V = K_vel
        User_W = 0.
    else:
        read_FSR()
        # FSR Inputs calibration: 
        Xin = FsrK * (Xin_temp - FsrZero)     # Values in [mV]
        for i in range (0,10):
            if Xin[i] < 0.:
                Xin[i] = 0.
        # Calculating the Center of pressure
        ox = np.sum(Rcenter*Xin) / (Xin[1] + Xin[2] + Xin[3] + Xin[4] + Xin[5] + Xin[6] + Xin[7] + Xin[8])
        if math.isnan(ox):
            ox = 0.
        Out_CP = round(ox, 6)
        FSR_execution()  # Runs the user input with Out_CP and returns Command_V and Command_W --> in 0-5k scale
        motor_v = 2*MAX_MOTOR_V*Command_V/5000 - MAX_MOTOR_V            # In [RPM]
        motor_w = (2*MAX_MOTOR_V/(DISTANCE_CW)*Command_W/5000 - MAX_MOTOR_V/(DISTANCE_CW)) / W_RATIO # In [RPM]
        User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),6)
        User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),6)

    if TIMING_MODE:
        FSR_time = round((time.clock() - t1),6)
        t1 = time.clock()

    if SHARED_MODE:
        rds_service()
        # Corrected_V = User_V
        # Corrected_W = User_W
    else:
        Corrected_V = User_V
        Corrected_W = User_W

    if TIMING_MODE:
        RDS_time = round((time.clock() - t1),6)
        t1 = time.clock()

    if COMPLIANCE_MODE:
        (compliant_V, compliant_W) = compliance_control.step(
            compliant_V, compliant_W,
            Corrected_V, Corrected_W,
            svr_data
        )
        Output_V = round(compliant_V,6)
        Output_W = round(compliant_W,6)
    else:
        Output_V = Corrected_V
        Output_W = Corrected_W

    if TIMING_MODE:
        Compliance_time = round((time.clock() - t1),6)
        t1 = time.clock()

    if math.isnan(Output_V):
        Output_V = 0.
    if math.isnan(Output_W):
        Output_W = 0.

    Output_V = clipper(Output_V, -MIN_SPEED, MAX_SPEED)
    Output_W = clipper(Output_W, -MAX_OMEGA, MAX_OMEGA)

    last_v = Output_V
    last_w = Output_W

    Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Output_V, Output_W)
    write_DA(Comand_DAC0, Comand_DAC1)
    if TIMING_MODE:
        DA_time = round((time.clock() - t1),6)
        Compute_time = round((time.clock() - t_start),6)
    # print ('FSR_read: %s, FSR_read: %s, FSR_read: %s, FSR_read: %s,')

    counter1 += 1  # for estiamting frequency


def make_header(frame_name):
        header = Header()
        header.frame_id = frame_name
        header.stamp = rospy.get_rostime()
        return header


def control_node():
    global Comand_DAC0, Comand_DAC1, Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3, Xin
    global RemoteE, ComError
    global DA_time, RDS_time, Compute_time, FSR_time, Compliance_time, extra_time,last_msg, time_msg, FULL_time
    global compliant_V, compliant_W, offset_ft_data, bumperModel, lp_filter, compliance_control, initialising_ft
    prevT = 0
    FlagEmergency=False
    # Call the calibration File
    # load_calibration()
    if COMPLIANCE_MODE:
        compliance_control = AdmittanceController(
            v_max=MAX_SPEED,
            omega_max=(MAX_OMEGA/W_RATIO),
            bumper_l=0.2425,
            bumper_R=0.33,
            Ts=1.0/100,
            Damping_gain=0.1,
            robot_mass=5,
            collision_F_max=45,
            activation_F=15,
            logger=logger
        )
    
    logger.init_topic("corr_velocity", "compliance", ["t", "v_user", "omega_user", "v_OA", "omega_OA", "v_compliance", "omega_compliance", "v_output", "omega_output"])
    if COMPLIANCE_MODE:
        logger.init_topic("svr", "compliance", ["t", "Fx", "Fy", "Mz"])
    if TIMING_MODE:
        logger.init_topic("timings", "compliance", ["t", "DA_time", "RDS_time", "Compute_time", "FSR_time", "Compliance_time", "Cycle_time"])

    ########### Starting Communication and MBED Board ###########
    ComError = conv.ReadChannel(6)
    if ComError<=THRESHOLD_V:
        enable_mbed()

    ########### Starting ROS Publishers ###########
    pub_emg = rospy.Publisher('qolo/emergency', Bool, queue_size=1)

    pub_twist = rospy.Publisher('qolo/twist', TwistStamped, queue_size=1)
    qolo_twist = TwistStamped()

    # pub_vel = rospy.Publisher('qolo/velocity', Float32MultiArray, queue_size=1)
    # dat_vel = Float32MultiArray()
    # dat_vel.layout.dim.append(MultiArrayDimension())
    # dat_vel.layout.dim[0].label = 'Velocities: last message, Input[2], Output[2]'
    # dat_vel.layout.dim[0].size = 5
    # dat_vel.data = [0]*4

    if DEBUG_MODE:
        pub_cor_vel = rospy.Publisher('qolo/corrected_velocity', Float32MultiArray, queue_size=1)
        dat_cor_vel = Float32MultiArray()
        dat_cor_vel.layout.dim.append(MultiArrayDimension())
        dat_cor_vel.layout.dim[0].label = 'Velocities: User[2] Corrected_OA[2] Corrected_Compliance[2] Output[2] RDS_dT'
        dat_cor_vel.layout.dim[0].size = 9
        dat_cor_vel.data = [0]*9

        pub_wheels = rospy.Publisher('qolo/wheels', Float32MultiArray, queue_size=1)
        dat_wheels = Float32MultiArray()
        dat_wheels.layout.dim.append(MultiArrayDimension())
        dat_wheels.layout.dim[0].label = 'Wheels Output'
        dat_wheels.layout.dim[0].size = 4
        dat_wheels.data = [0]*4

        pub_user = rospy.Publisher('qolo/user_input', Float32MultiArray, queue_size=1)
        dat_user = Float32MultiArray()
        dat_user.layout.dim.append(MultiArrayDimension())
        dat_user.layout.dim[0].label = 'FSR_read'
        dat_user.layout.dim[0].size = 11
        dat_user.data = [0]*11
        
        if COMPLIANCE_MODE:
            pub_compliance_bumper_loc = rospy.Publisher('qolo/compliance/bumper_loc', Float32MultiArray, queue_size=1)
            dat_compliance_bumper_loc = Float32MultiArray()
            dat_compliance_bumper_loc.layout.dim.append(MultiArrayDimension())
            dat_compliance_bumper_loc.layout.dim[0].label = 'F_mag bumper_theta bumper_h'
            dat_compliance_bumper_loc.layout.dim[0].size = 3
            dat_compliance_bumper_loc.data = [0]*3
    
    ########### Starting ROS Node ###########
    rospy.init_node('qolo_control', anonymous=True)
    rate = rospy.Rate(200) #  100 [Hz]

    if COMPLIANCE_MODE:
        ftsub = rospy.Subscriber("qolo/compliance/svr",WrenchStamped,ft_sensor_callback, queue_size=1)
        rospy.loginfo('Starting Compliant Mode')
    else:
        print('Starting WITHOUT FT Sensing')
    
    if JOYSTICK_MODE:
        sub_remote = rospy.Subscriber("qolo/remote_commands", Float32MultiArray, callback_remote, queue_size=1)
        control_type = 'joystick'
        print('Subscribed to JOYSTICK Mode')
    elif REMOTE_MODE:
        sub_remote = rospy.Subscriber("qolo/remote_commands", Float32MultiArray, callback_remote, queue_size=1)
        control_type = 'remote'
        print('Subscribed to REMOTE Mode')
    else:
        control_type = 'embodied'
    
    if SHARED_MODE:
        print('STARTING SHARED CONTROL MODE')
    else:
        print('STARTING MANUAL MODE')

    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    while not rospy.is_shutdown():
        prevT = time.clock()

        control()   # Function of control for Qolo
        # # Checking emergency inputs
        RemoteE = conv.ReadChannel(7) # THRESHOLD_V - 1 # conv.ReadChannel(7)
        ComError = conv.ReadChannel(6) # THRESHOLD_V + 1 # conv.ReadChannel(6)
        # print('Comerror', ComError)
        if ComError<=THRESHOLD_V:
            enable_mbed()
        if RemoteE >= THRESHOLD_V:
            print('RemoteE', RemoteE)
            FlagEmergency=True
            time_msg=0.
            while FlagEmergency:
                pub_emg.publish(FlagEmergency)
                conv.SetChannel(3, 0)
                conv.SetChannel(2, 0)
                conv.SetChannel(0, ZERO_LW)
                conv.SetChannel(1, ZERO_RW)
                ResetFSR = conv.ReadChannel(5)
                if ResetFSR >= THRESHOLD_V:
                    print('ResetFSR ', ResetFSR)
                    FlagEmergency=False
                    pub_emg.publish(FlagEmergency)
                    enable_mbed()
                    time_msg=0.
                time.sleep(0.1)
        
        qolo_twist.header = make_header("tf_qolo")
        qolo_twist.twist.linear.x = Output_V
        qolo_twist.twist.angular.z = Output_W

        # dat_vel.data = [time_msg, User_V, User_W, Output_V, Output_W]
        
        if DEBUG_MODE:
            dat_cor_vel.data = [User_V, User_W, Corrected_V, Corrected_W, compliant_V, compliant_W, Output_V, Output_W, RDS_time]
            dat_wheels.data = [Send_DAC0, Send_DAC1, Send_DAC2, Send_DAC3]
            dat_user.data = [Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9],Out_CP]
            if COMPLIANCE_MODE:
                dat_compliance_bumper_loc.data = bumper_loc

        if TIMING_MODE:
            cycle_T = time.clock() - prevT

        # ----- CSV Logs -----
        logger.log('corr_velocity',
                   User_V, User_W,
                   Corrected_V, Corrected_W,
                   compliant_V, compliant_W,
                   Output_V, Output_W)
        
        if COMPLIANCE_MODE:
            compliance_control.log()
            logger.log('svr', *svr_data)
        
        if TIMING_MODE:
            logger.log('timings', DA_time, RDS_time, Compute_time, FSR_time, Compliance_time, cycle_T, FULL_time)

        # ----- Publish ROS topics -----
        pub_emg.publish(FlagEmergency)
        pub_twist.publish(qolo_twist)

        # pub_vel.publish(dat_vel)

        if DEBUG_MODE:
            pub_cor_vel.publish(dat_cor_vel)
            pub_wheels.publish(dat_wheels)
            pub_user.publish(dat_user)
            if COMPLIANCE_MODE:
                pub_compliance_bumper_loc.publish(dat_compliance_bumper_loc)

        FULL_time = time.clock() - prevT
        compliance_control.update_Ts(FULL_time)

        rate.sleep()

    logger.exit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass
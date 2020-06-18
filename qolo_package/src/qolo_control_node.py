#! /usr/bin/env python
#########  Qolo Main Code for Shared / Embodied / Remore Control ##########
##### Author: Diego F. Paez G.
##### Embodied sensirg: Chen Yang
##### Data: 2019/10/01

import HighPrecision_ADDA_Double as converter
import time
import math
import os
from itertools import groupby
# import threading
# import RPi.GPIO as GPIO
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
# from scipy import signal
import rospy
import threading
from prediction_model import BumperModel
from filters import MultiLowPassFilter

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from rds_network_ros.srv import *
# from builtins import PermissionError

# FLAG for fully manual control (TRUE) or shared control (FALSE)
#Tonado server port
try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    rospy.logerr("Cannot set niceness for the process...")
    rospy.logerr("Run the script as sudo...")
K_vel = 0.5
CONSTANT_VEL = False
# For testing collision avoidance 
SHARED_MODE = False
# For testing collision control
COMPLIANCE_FLAG = True
# For using remote app Joystick
JOYSTICK_MODE = True
# For using DS or trajectory tracking
REMOTE_MODE = True
# For zero output to the wheels
TESTING_MODE = False

PORT = 8080
control_type ='embodied'
threadLock = threading.Lock()
FLAG_debug = True
Stop_Thread_Flag = False

conv = converter.AD_DA()

# coefficient for vmax and wmax(outout curve)
forward_coefficient = 1
left_turning_coefficient = 1
right_turning_coefficient = 1
backward_coefficient = 0.5

# Global Constatns for Communication
# DAC0 --> Left Wheel Velocity
# DAC1 --> Right Wheel Velocity
# DAC2 --> Enable Qolo Motion
THRESHOLD_V = 1500;
ZERO_LW = 2600 #2750;
ZERO_RW = 2500 #2650;
High_DAC = 5000;
MBED_Enable = mraa.Gpio(36) #11 17
MBED_Enable.dir(mraa.DIR_OUT)

GEAR = 12.64 # Gear ratio
DISTANCE_CW = 0.548/2  # DISTANCE_CW bettween two wheels
RADIUS = 0.304/2 # meter

MAX_SPEED = 1.5 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
MIN_SPEED = MAX_SPEED*backward_coefficient
MAX_OMEGA = 4.124
W_RATIO = 3.5 # Ratio of the maximum angular speed (232 deg/s)

MAX_MOTOR_V = 1200.0 # max motor speed: 1200 rpm

MAX_WHEEL_VEL = (95.0/60.0)*(2*np.pi)
WHEEL_ACC = (((4.0/60.0)*(2*np.pi))/GEAR)/(1.0/400.0) # Maximum wheel acceleration = 13.2557 rad/s2
LINEAR_ACC = WHEEL_ACC * RADIUS # Maximum Robot's linear acceleration = 2.0149 m/s2
ANGULAR_ACC = LINEAR_ACC / DISTANCE_CW # Maximum Robot's angular acceleration = 7.3535 rad/s2 

#########################################################
############ Setting for the RDS service ################
#########################################################
LRF_points_Flag = True
# y_coordinate_of_reference_point_for_command_limits = 0.5;
# Gain to this point
weight_scaling_of_reference_point_for_command_limits = 0.;
# Some gain for velocity after proximity reaches limits
tau = 1.5;
# Minimal distance to obstacles
delta = 0.05;
# Some reference for controlling the non-holonomic base
control_point = 0.18

max_linear = MAX_SPEED;
min_linear = -MIN_SPEED;
absolute_angular_at_min_linear = 0.;
absolute_angular_at_max_linear = 0.;
absolute_angular_at_zero_linear = MAX_OMEGA/W_RATIO;
linear_acceleration_limit = 1.5
angular_acceleration_limit = 1.5


#########################################################
############ Setting for Compliant Control ##############
#########################################################
compliant_V =0.
compliant_W =0.

bumper_l = 0.2425      # (210+32.5) mm
bumper_R = 0.33 # 330 mm
Ts = 1.0/50    # 100 Hz
control_time = 0.1
Damping_gain = 10         # 1 N-s/m 
robot_mass = 10        # 120 kg
collision_F_max = 200 # [N]

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
bumper_loc = np.zeros((4,))

# Prediction Models
bumperModel = None
lp_filter = None

feasible = 0
Output_V = 0.;
Output_W = 0.;
Corrected_V = 0.;
Corrected_W = 0.;
Remote_V = 0.;
Remote_W = 0.;
FlagRemote = False
last_msg = 0.
time_msg = 0.
Count_msg_lost = 0

last_v = 0.;
last_w = 0.;
cycle=0.

Command_V = 2500
Command_W = 2500
Comand_DAC0 = 0
Comand_DAC1 = 0
Send_DAC0 = 0
Send_DAC1 = 0 
# rpm_L = 0;
# rpm_R = 0;

# Global variables for logging
DA_time = 0.
RDS_time = 0.
Compute_time = 0. 
FSR_time = 0.
User_V = 0.;
User_W = 0.;
Out_CP = 0.;
Out_F = 0.;
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
    # global Stop_Thread_Flag
    # MBED_Enable = mraa.Gpio(36) #11 17
    # MBED_Enable.dir(mraa.DIR_OUT)
    MBED_Enable.write(0)
    conv.SET_DAC2(0, conv.data_format.voltage)
    conv.SET_DAC3(0, conv.data_format.voltage)
    conv.SET_DAC0(0, conv.data_format.voltage)
    conv.SET_DAC1(0, conv.data_format.voltage)
    # Stop_Thread_Flag = True
    # cleanup_stop_thread()
    print('----> You chose to interrupt')
    quit()

class FSR_thread (threading.Thread):
    def __init__(self, name, counter):
        threading.Thread.__init__(self)
        self.threadID = counter
        self.name = name
        self.counter = counter
    def run(self):
        # print("\nStarting " + self.name)
        # Acquire lock to synchronize thread
        while True:
            have_it = threadLock.acquire()
            try:
                if have_it:
                    read_FSR()
                    threadLock.release()
                    have_it = 0
                    user_input_thread()
            finally:
                if have_it:
                    threadLock.release()
                # Release lock for the next thread

            if Stop_Thread_Flag:
                break

        print("Exiting " + self.name)

    def get_id(self): 
        # returns id of the respective thread 
        if hasattr(self, '_thread_id'): 
            return self._thread_id 
        for id, thread in threading._active.items(): 
            if thread is self: 
                return id

    def raise_exception(self): 
        thread_id = self.get_id() 
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 
              ctypes.py_object(SystemExit)) 
        if res > 1: 
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
            print('Exception raised: Stopped User_input_thread')


def callback_remote(data):
    global Remote_V, Remote_W, FlagRemote, time_msg
    # temp_dat = Float32MultiArray()
    # temp_dat.layout.dim.append(MultiArrayDimension())
    # temp_dat.layout.dim[0].size = 2
    # temp_dat.data = [0]*2
    # print(time_msg)
    if time_msg != 0:
        Remote_V = round(data.data[1],8)
        Remote_W =  round(data.data[2],8)
        FlagRemote = True
        time_msg = data.data[0]
    time_msg = data.data[0]

## Compliant control functions
def ft_sensor_callback(data):
    # data.wrench.serialize_numpy(raw_ft_data, np)
    global raw_ft_data, filtered_ft_data, init_ft_data, initialising_ft
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
    # else:
    #     filter_data['Fx'] = np.append(filter_data['Fx'], _x.force.x)
    #     filter_data['Fy'] = np.append(filter_data['Fy'], _x.force.y)
    #     filter_data['Fz'] = np.append(filter_data['Fz'], _x.force.z)
    #     filter_data['Mx'] = np.append(filter_data['Mx'], _x.torque.x)
    #     filter_data['My'] = np.append(filter_data['My'], _x.torque.y)
    #     filter_data['Mz'] = np.append(filter_data['Mz'], _x.torque.z)

    #     fs = 400 #sampling frequency
    #     fc = 30 # Cut-off frequency of the filter
    #     w = fc / (fs / 2) # Normalize the frequency
    #     b, a = signal.butter(5, w, 'low')
    #     fx_low = signal.lfilter(b, a, filter_data['Fx']) #Forward filter
    #     fy_low = signal.lfilter(b, a, filter_data['Fy']) #Forward filter
    #     fz_low = signal.lfilter(b, a, filter_data['Fz']) #Forward filter
    #     tx_low = signal.lfilter(b, a, filter_data['Mx']) #Forward filter
    #     ty_low = signal.lfilter(b, a, filter_data['My']) #Forward filter
    #     tz_low = signal.lfilter(b, a, filter_data['Mz']) #Forward filter
    #     filtered_ft_data = np.array([
    #         fx_low,
    #         fy_low,
    #         fz_low,
    #         tx_low,
    #         ty_low,
    #         tz_low,
    #         ])


def damper_correction(ft_data):
    # Correcting based on trained SVR damping model
    global bumperModel, bumper_loc
    # corr_ft_data = bumperModel.predict(ft_data)
    # ft_data_temp = ft_data
    (Fx, Fy, Mz) = bumperModel.predict(np.reshape(ft_data, (1,-1)))
    h = 0.1
    (a, b, c) = (Fx, Fy, Mz/bumper_R)
    temp = a**2 + b**2 - c**2
    if temp > 0:
        theta = np.real(-1j * np.log(
            (c + 1j*np.sqrt(temp)) /
            (a + 1j*b)
        ))
    else:
        theta = np.real(-1j * np.log(
            (c - np.sqrt(-temp)) /
            (a + 1j*b)
        ))

    Fmag = Fx*np.sin(theta) + Fy*np.cos(theta)
    
    rospy.loginfo(
        "\n\tFx = {}\n\tFy = {}\n\tMz = {}\n\ttheta = {}\n\tFmag = {}\n-------------------------\n".format(
            Fx, Fy, Mz, theta, Fmag
        )
    )

    bumper_loc[0] = Fmag
    bumper_loc[1] = theta
    bumper_loc[2] = h

    return (Fx, Fy, Mz, Fmag, h, theta)


def compliance_control(v_prev, omega_prev, v_cmd, omega_cmd, Fmag, h, theta):
    global control_time, bumper_loc
    # F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x
    # And set reference to 0 and discretize w/ ZOH
    stheta = math.sin(theta)    # Small optimization
    ctheta = math.cos(theta)    # Small optimization

    # Position wrt center of rotatiion
    O = math.sqrt((bumper_R*stheta)**2 + (bumper_l + bumper_R*ctheta)**2 )
    beta = math.atan2(bumper_R * stheta, bumper_l + bumper_R * ctheta)

    sbeta = math.sin(beta)      # Small optimization
    cbeta = math.cos(beta)      # Small optimization

    # Admittance Control
    Ts_control = round((time.clock() - control_time),4)
    Ts_control = Ts
    control_time = time.clock()
    
    a = ctheta
    b = O * (stheta*cbeta - ctheta*sbeta)
    
    v_eff_prev = (a * v_prev) + (b * omega_prev)
    v_eff_cmd  = (a * v_cmd)  + (b * omega_cmd)

    v_eff_dot = (-Fmag - Damping_gain*v_eff_prev) / robot_mass
    v_eff = v_eff_dot * Ts_control + v_eff_cmd

    # # Calculate new v and omega
    # c_prev = (-b * v_prev) + (a * omega_prev)
    # den = a**2 - b**2
    # v = (a*v_eff - b*c_prev) / den
    # omega = (-b*v_eff + a*c_prev) / den

    # Calculate new v and omega in parameterized form
    v_max = MAX_SPEED
    omega_max = (MAX_OMEGA/W_RATIO)
    
    a = -1.0 / v_max
    b = (stheta*cbeta - ctheta*sbeta) / omega_max

    v_eff_max = (-collision_F_max * Ts_control) / robot_mass
    V = v_eff / v_eff_max

    # Ensure non-zero 'a' and 'b'
    eps = 0.01
    if (abs(a) < eps):
        # return (v_prev, v_eff/b)
        return (v_cmd, V/b)
    if (abs(b) < eps):
        # return (v_eff/a, omega_prev)
        return (V/a, omega_cmd)

    _ = V - a*omega_cmd / b
    if _ > omega_max:
        t_max = (omega_max - omega_cmd) / (_ - omega_cmd)
    elif _ < -omega_max:
        t_max = (-omega_max - omega_cmd) / (_ - omega_cmd)
    else:
        t_max = 1.0

    _ = V - b*v_cmd / a
    if _ > v_max:
        t_min = (v_max - omega_cmd) / (_ - omega_cmd)
    elif _ < -v_max:
        t_min = (-v_max - omega_cmd) / (_ - omega_cmd)
    else:
        t_min = 0.0

    __from_range = [0.0, np.pi]
    __to_range = [t_min, t_max]
    __x = np.abs(theta)
    t = __to_range[0] + ((__x - __from_range[0]) * (__to_range[1]-__to_range[0]) / (__from_range[1]-__from_range[0]))
    bumper_loc[3] = t

    # v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
    # omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev

    v = t * v_cmd + (1-t) * (V - b*omega_cmd) / a
    omega = t * (V - a*v_cmd) / b + (1-t) * omega_cmd
    return (v, omega)
    

# read data from ADDA board
def read_FSR():
    global Xin_temp, RemoteE, ComError
    # Checking emergency inputs
    # RemoteE = conv.ReadChannel(7, conv.data_format.voltage)
    # ComError = conv.ReadChannel(6, conv.data_format.voltage)
     # ADC_Value[0]*5.0/0x7fffff
    Xin_temp[0] = round(conv.ReadChannel(5, conv.data_format.voltage),4)
    Xin_temp[1] = round(conv.ReadChannel(15, conv.data_format.voltage),4)
    Xin_temp[2] = round(conv.ReadChannel(14, conv.data_format.voltage),4)
    Xin_temp[3] = round(conv.ReadChannel(13, conv.data_format.voltage),4)
    Xin_temp[4] = round(conv.ReadChannel(12, conv.data_format.voltage),4)
    Xin_temp[5] = round(conv.ReadChannel(11, conv.data_format.voltage),4)
    Xin_temp[6] = round(conv.ReadChannel(10, conv.data_format.voltage),4)
    Xin_temp[7] = round(conv.ReadChannel(9, conv.data_format.voltage),4)
    Xin_temp[8] = round(conv.ReadChannel(8, conv.data_format.voltage),4)
    Xin_temp[9] = round(conv.ReadChannel(4, conv.data_format.voltage),4)

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
        
def user_input_thread():
    global FSR_time, t2, Xin, Xin_temp, FsrZero, FsrK, Out_CP, User_V, User_W, Command_V, Command_W, Read_Flag

    # read_FSR()
    Xin = FsrK* (round(Xin_temp,4) - FsrZero)     # Values in [mV]
    # Calculating the Center of pressure
    ox = np.sum(Rcenter*Xin) / (Xin[1] + Xin[2] + Xin[3] + Xin[4] + Xin[5] + Xin[6] + Xin[7] + Xin[8])
    Out_CP = round(ox, 4)
    # Runs the user input and returns Command_V and Command_W --> in 0-5k scale
    FSR_execution()
    motor_v = 2*MAX_MOTOR_V*Command_V/5000 - MAX_MOTOR_V            # In [RPM]
    motor_w = (2*MAX_MOTOR_V/(DISTANCE_CW)*Command_W/5000 - MAX_MOTOR_V/(DISTANCE_CW)) / W_RATIO # In [RPM]

    # Start lock
    User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),4)
    User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),4)
    # if Stop_Thread_Flag:
    #     break
    
    if FLAG_debug:
        FSR_time = round((time.clock() - t2),4)
        t2 = time.clock()


def enable_mbed():
    # threadLock.acquire()

    MBED_Enable.write(0)
    time.sleep(1)
    MBED_Enable.write(1)
    print("Initiating MBED")
    time.sleep(2)

    StartFlag = 1
    conv.SET_DAC2(High_DAC, conv.data_format.voltage)
    conv.SET_DAC0(ZERO_LW, conv.data_format.voltage)
    conv.SET_DAC1(ZERO_RW, conv.data_format.voltage)

    ComError = conv.ReadChannel(6, conv.data_format.voltage)    
    while StartFlag:
        print("Waiting MBED_Enable")
        time.sleep(0.5)
        ComError = conv.ReadChannel(6, conv.data_format.voltage)
        print("ComError = ",ComError)
        if ComError>THRESHOLD_V:
            StartFlag=0

    time.sleep(1)  
    # threadLock.release()

def transformTo_Lowevel(Desired_V, Desired_W):
    # A function to transform linear and angular velocities to output commands
    # print('received ', Command_V, Command_W)
    global DISTANCE_CW, RADIUS, User_V, User_W, MAX_SPEED, GEAR, MAX_MOTOR_V

    # These lines should be commented to execute the RDS output
    # motor_v = 2*MAX_MOTOR_V*Command_V/5000 - MAX_MOTOR_V            # In [RPM]
    # motor_w = (2*MAX_MOTOR_V/(DISTANCE_CW)*Command_W/5000 - MAX_MOTOR_V/(DISTANCE_CW)) / W_RATIO # In [RPM]
    # User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),4)
    # User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),4)

    # Using the desired velocity (linearn adn angular) --> transform to motor speed

    wheel_L = Desired_V - (DISTANCE_CW * Desired_W)    # Output in [m/s]
    wheel_R = Desired_V + (DISTANCE_CW * Desired_W)    # Output in [m/s]
    # print ('Wheels Vel =', wheel_L, wheel_R)

    # motor_v = round(((Desired_V*GEAR)/RADIUS)/(np.pi/30),8) 
    # motor_w = round(((Desired_W*GEAR)/RADIUS)/(np.pi/30),8) 

    # rpm_L = motor_v - DISTANCE_CW*motor_w
    # rpm_R = motor_v + DISTANCE_CW*motor_w
    # Transforming from rad/s to [RPM]
    motor_l = (wheel_L/RADIUS) * GEAR *(30/np.pi)
    motor_r = (wheel_R/RADIUS) * GEAR *(30/np.pi)
    # print ('Motor Vel =', motor_l, motor_r)    
    # Transforming velocities to mV [0-5000]
    Command_L = round( (ZERO_LW + 5000*motor_l/2400), 6)
    Command_R = round ( (ZERO_RW + 5000*motor_r/2400), 6)
    

    return Command_L, Command_R


def write_DA(Write_DAC0,Write_DAC1):
    global Send_DAC0, Send_DAC1

    # ADC Board output in mV [0-5000]
    if Write_DAC0 > 4800:
        Send_DAC0 = 4800
    elif Write_DAC0 < 200:
        Send_DAC0 = 200
    else:
        Send_DAC0 = Write_DAC0

    if Write_DAC1 > 4800:
        Send_DAC1 = 4800
    elif Write_DAC1 < 200:
        Send_DAC1 = 200
    else:
        Send_DAC1 = Write_DAC1

    if TESTING_MODE:
        Send_DAC0 = ZERO_LW;
        Send_DAC1 = ZERO_RW;
        send_DAC2 = 0
    else:
        send_DAC2 = High_DAC;

    # threadLock.acquire()
    conv.SET_DAC0(Send_DAC0, conv.data_format.voltage)
    conv.SET_DAC1(Send_DAC1, conv.data_format.voltage)
    conv.SET_DAC2(send_DAC2, conv.data_format.voltage)
    # threadLock.release()


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
        # max_linear = MAX_SPEED;
        # min_linear = -MIN_SPEED;
        # absolute_angular_at_min_linear = 0.;
        # absolute_angular_at_max_linear = 0.;
        # absolute_angular_at_zero_linear = MAX_OMEGA/W_RATIO;
        # linear_acceleration_limit = 1.1
        # angular_acceleration_limit = 1.5

        request.nominal_command.linear = User_V;
        request.nominal_command.angular = User_W;
        request.capsule_center_front_y = 0.051;
        request.capsule_center_rear_y = -0.50;
        request.capsule_radius = 0.45;
        
        request.reference_point_y = control_point;

        request.rds_tau = tau;  # Time horizon for velocity obstacles
        request.rds_delta = delta;
        request.vel_lim_linear_min = min_linear;
        request.vel_lim_linear_max = max_linear;
        request.vel_lim_angular_abs_max = absolute_angular_at_zero_linear;
        request.vel_linear_at_angular_abs_max = 0.2;
        request.acc_limit_linear_abs_max = linear_acceleration_limit;
        request.acc_limit_angular_abs_max = angular_acceleration_limit;

        # // shall rds consider lrf measurements?
        request.lrf_point_obstacles = LRF_points_Flag;
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
        # // shall rds work with bounding circles or find per object the closest incircle in the capsule?
        # // any integer n > 2 : use n bounding circles
        # // any integer <= 2 : use local capsule incircles
        request.bounding_circles = 2;

        if cycle==0:
            delta_time = 0.005;
        else:
            delta_time = time.clock() - cycle;

        request.dt = 0.01 #delta_time

        response = RDS(request)
        Corrected_V = round(response.corrected_command.linear,6)
        Corrected_W = round(response.corrected_command.angular,6)

        cycle = time.clock()
    except:
        Corrected_V = 0.
        Corrected_W = 0.

def mds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible, Corrected_V, Corrected_W, control_point
    # print "Waiting for MDS Service"

    rospy.wait_for_service('rds_velocity_command_correction')
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()

    request.nominal_command.linear = User_V;
    request.nominal_command.angular = User_W;
    request.capsule_center_front_y = 0.05;
    request.capsule_center_rear_y = -0.5;
    request.capsule_radius = 0.45;
    request.reference_point_y = control_point;
    request.rds_tau = 1.5;
    request.rds_delta = 0.05;
    request.vel_lim_linear_min = 0.5;
    request.vel_lim_linear_max = 1.5;
    request.vel_lim_angular_abs_max = 1.0;
    request.vel_linear_at_angular_abs_max = 0.2;
    request.acc_limit_linear_abs_max = 0.5;
    request.acc_limit_angular_abs_max = 0.5;
    if cycle==0:
        delta_time = 0.005;
    else:
        delta_time = time.clock() - cycle;
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
    global counter1, compliant_V, compliant_W, raw_ft_data, ft_data, svr_data, offset_ft_data
    global DA_time, RDS_time, Compute_time, FSR_time
    
    # Replace with a node subsription
    global Xin, Xin_temp, FsrZero, FsrK, Out_CP
    if FLAG_debug:
        t1 = time.clock()

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
    elif CONSTANT_VEL:
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
        Out_CP = round(ox, 6);

        FSR_execution()  # Runs the user input with Out_CP and returns Command_V and Command_W --> in 0-5k scale
        motor_v = 2*MAX_MOTOR_V*Command_V/5000 - MAX_MOTOR_V            # In [RPM]
        motor_w = (2*MAX_MOTOR_V/(DISTANCE_CW)*Command_W/5000 - MAX_MOTOR_V/(DISTANCE_CW)) / W_RATIO # In [RPM]
        User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),6)
        User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),6)


    if FLAG_debug:
        FSR_time = round((time.clock() - t1),4)
        t1 = time.clock()

    if SHARED_MODE:
        rds_service()
        # Corrected_V = User_V
        # Corrected_W = User_W
    else:
        Corrected_V = User_V
        Corrected_W = User_W

    if FLAG_debug:
        RDS_time = round((time.clock() - t1),4)
        t1 = time.clock()

    if COMPLIANCE_FLAG:
        # ft_data = lp_filter.filter(raw_ft_data - offset_ft_data)
        ft_data = (raw_ft_data - offset_ft_data)
        [Fx, Fy, Mz, Fmag, h, theta] = damper_correction(ft_data)
        svr_data[0] = Fx
        svr_data[1] = Fy
        svr_data[2] = Mz
        if abs(Fmag) > 15:
            [compliant_V, compliant_W] = compliance_control(compliant_V, compliant_W, Corrected_V, Corrected_W, Fmag, h, theta)
        else:
            (compliant_V, compliant_W) = (Corrected_V, Corrected_W)
        Output_V = round(compliant_V,6)
        Output_W = round(compliant_W,6)
    else:
        Output_V = Corrected_V
        Output_W = Corrected_W
    
    # Debugging the speed controller
    # if counter1 < 20:
    #     Comand_DAC0 = 4000
    #     Comand_DAC1 = 4000
    # else:
    #     Comand_DAC0 = ZERO_LW
    #     Comand_DAC1 = ZERO_RW

    if Output_V > MAX_SPEED:
        Output_V = MAX_SPEED
    elif Output_V < -MIN_SPEED:
        Output_V = -MIN_SPEED

    if Output_W > MAX_OMEGA:
        Output_W = MAX_OMEGA
    elif Output_W < -MAX_OMEGA:
        Output_W = -MAX_OMEGA

    last_v = Output_V
    last_w = Output_W

    Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Output_V, Output_W)
    write_DA(Comand_DAC0, Comand_DAC1)
    if FLAG_debug:
        DA_time = round((time.clock() - t1),4)
    # print ('FSR_read: %s, FSR_read: %s, FSR_read: %s, FSR_read: %s,')

    counter1 += 1  # for estiamting frequency


def control_node():
    global Comand_DAC0, Comand_DAC1, Send_DAC0, Send_DAC1, Xin
    global RemoteE, ComError
    global DA_time, RDS_time, Compute_time, FSR_time, extra_time,last_msg, time_msg
    global compliant_V, compliant_W, offset_ft_data, bumperModel, lp_filter
    prevT = 0
    FlagEmergency=False
    # threadLock = threading.Lock()
    # Setting ROS Node
    
    # Call the calibration File
    # load_calibration()
    if COMPLIANCE_FLAG:
        rospy.loginfo("loading SVR models")
        bumperModel = BumperModel()
        rospy.loginfo("SVR models loaded")
        lp_filter = MultiLowPassFilter(size=6)


    ########### Starting Communication and MBED Board ###########

    ComError = conv.ReadChannel(6, conv.data_format.voltage)
    if ComError<=THRESHOLD_V:
        enable_mbed()

    ########### creating threads  ##############
    # try:
    #     thread_user = FSR_thread("user_input", 1)
    #     Stop_Thread_Flag = False
    #     # thread_user = threading.Thread(target=user_input_thread)
    #     print "FSR Thread started"
    # except:
    #    print "Error: unable to start FSR thread"
    # start input thread
    # thread_user.run(threadLock)
    # thread_user.start()

    ########### Starting ROS Node ###########
    # pub = rospy.Publisher('qolo', String, queue_size=1)
    # rospy.init_node('qolo_control', anonymous=True)
    # rate = rospy.Rate(50) #  20 hz

    ########### Starting ROS Node ###########
    pub_wheels = rospy.Publisher('qolo/wheels', Float32MultiArray, queue_size=1)
    pub_twist = rospy.Publisher('qolo/twist', TwistStamped, queue_size=1)
    pub_vel = rospy.Publisher('qolo/velocity', Float32MultiArray, queue_size=1)
    pub_cor_vel = rospy.Publisher('qolo/corrected_velocity', Float32MultiArray, queue_size=1)
    pub_emg = rospy.Publisher('qolo/emergency', Bool, queue_size=1)
    pub_user = rospy.Publisher('qolo/user_input', Float32MultiArray, queue_size=1)
    pub_compliance_raw = rospy.Publisher('qolo/compliance/raw', WrenchStamped, queue_size=1)
    pub_compliance_svr = rospy.Publisher('qolo/compliance/svr', WrenchStamped, queue_size=1)
    pub_compliance_bumper_loc = rospy.Publisher('qolo/compliance/bumper_loc', Float32MultiArray, queue_size=1)
    
    pub_mess = rospy.Publisher('qolo/message', String, queue_size=1)
    rospy.init_node('qolo_control', anonymous=True)
    rate = rospy.Rate(50) #  100 hz

    
    def make_header(name):
        header = Header()
        header.frame_id = name
        header.stamp = rospy.get_rostime()
        # header.stamp = rospy.Time.now()
        return header

    dat_user = Float32MultiArray()
    dat_user.layout.dim.append(MultiArrayDimension())
    dat_user.layout.dim[0].label = 'FSR_read'
    dat_user.layout.dim[0].size = 11
    dat_user.data = [0]*11

    qolo_twist = TwistStamped()
    qolo_twist.header = make_header("tf_qolo")
    qolo_twist.twist.linear.x = 0
    qolo_twist.twist.linear.y = 0
    qolo_twist.twist.linear.z = 0
    qolo_twist.twist.angular.x = 0
    qolo_twist.twist.angular.y = 0
    qolo_twist.twist.angular.z = 0

    dat_vel = Float32MultiArray()
    dat_vel.layout.dim.append(MultiArrayDimension())
    dat_vel.layout.dim[0].label = 'Velocities: last message, Input[2], Output[2]'
    dat_vel.layout.dim[0].size = 5
    dat_vel.data = [0]*4

    dat_cor_vel = Float32MultiArray()
    dat_cor_vel.layout.dim.append(MultiArrayDimension())
    dat_cor_vel.layout.dim[0].label = 'Velocities: User[2] Corrected_OA[2] Corrected_Compliance[2] RDS_dT'
    dat_cor_vel.layout.dim[0].size = 7
    dat_cor_vel.data = [0]*7

    dat_wheels = Float32MultiArray()
    dat_wheels.layout.dim.append(MultiArrayDimension())
    dat_wheels.layout.dim[0].label = 'Wheels Output'
    dat_wheels.layout.dim[0].size = 2
    dat_wheels.data = [0]*2

    dat_compliance_raw = WrenchStamped()
    dat_compliance_svr = WrenchStamped()

    dat_compliance_bumper_loc = Float32MultiArray()
    dat_compliance_bumper_loc.layout.dim.append(MultiArrayDimension())
    dat_compliance_bumper_loc.layout.dim[0].label = 'Bumper: F_mag theta h ; param'
    dat_compliance_bumper_loc.layout.dim[0].size = 4
    dat_compliance_bumper_loc.data = [0]*4


    if COMPLIANCE_FLAG:
        ftsub = rospy.Subscriber("/rokubi_node_front/ft_sensor_measurements",WrenchStamped,ft_sensor_callback, queue_size=1)
        start_time = time.time()
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
        sub_remote = rospy.Subscriber("qolo/remote_joystick", Float32MultiArray, callback_remote, queue_size=1)
        control_type = 'joystick'
    elif REMOTE_MODE:
        sub_remote = rospy.Subscriber("qolo/remote_commands", Float32MultiArray, callback_remote, queue_size=1)
        control_type = 'remote'
    else:
        control_type = 'embodied'
    
    if SHARED_MODE:
        print('STARTING SHARED CONTROL MODE')
    else:
        print('STARTING MANUAL MODE')

    while not rospy.is_shutdown():

        control()   # Function of control for Qolo
        # # Checking emergency inputs
        # threadLock.acquire()
        RemoteE = conv.ReadChannel(7, conv.data_format.voltage)
        ComError = conv.ReadChannel(6, conv.data_format.voltage)
        # threadLock.release()
        # print('Comerror', ComError)
        if ComError<=THRESHOLD_V:
            enable_mbed()
        if RemoteE >= THRESHOLD_V:
            print('RemoteE', RemoteE)
            FlagEmergency=True
            time_msg=0.
            # threadLock.acquire()
            while FlagEmergency:
                pub_emg.publish(FlagEmergency)
                conv.SET_DAC2(0, conv.data_format.voltage)
                conv.SET_DAC0(ZERO_LW, conv.data_format.voltage)
                conv.SET_DAC1(ZERO_RW, conv.data_format.voltage)
                ResetFSR = conv.ReadChannel(5, conv.data_format.voltage)
                if ResetFSR >= THRESHOLD_V:
                    print('ResetFSR ', ResetFSR)
                    FlagEmergency=False
                    pub_emg.publish(FlagEmergency)
                    enable_mbed()
                    time_msg=0.
                time.sleep(0.1)
                # threadLock.release()

        cycle_T = time.clock() - prevT
        prevT = time.clock()
        now = datetime.datetime.now()
        current_time = now.strftime("%H:%M:%S")
        # RosMassage = "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s" % (current_time, cycle_T, RDS_time,Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9],Out_CP,Send_DAC0, Send_DAC1, User_V, User_W, feasible, Output_V, Output_W)
        # RosMassage = "%s %s %s %s %s %s %s %s" % (cycle_T, RDS_time, DA_time, feasible, User_V, User_W, round(Output_V,4), round(Output_W,4) )
        RosMassage = "%s %s %s %s %s" % (User_V, User_W, Output_V, Output_W, RDS_time)
        dat_wheels.data = [Send_DAC0, Send_DAC1]
        dat_vel.data = [time_msg, User_V, User_W, Output_V, Output_W]
        
        qolo_twist.header = make_header("tf_qolo")
        qolo_twist.twist.linear.x = Output_V
        qolo_twist.twist.angular.z = Output_W
        
        dat_cor_vel.data = [User_V, User_W, Corrected_V, Corrected_W, compliant_V, compliant_W, RDS_time]
        dat_user.data = [Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9],Out_CP]
        

        #### CHANGE THIS TO WRENCH TYPE ##############

        dat_compliance_raw.header = make_header("tf_ft_front")
        dat_compliance_raw.wrench.force.x = ft_data[0]
        dat_compliance_raw.wrench.force.y = ft_data[1]
        dat_compliance_raw.wrench.force.z = ft_data[2]
        dat_compliance_raw.wrench.torque.x = ft_data[3]
        dat_compliance_raw.wrench.torque.y = ft_data[4]
        dat_compliance_raw.wrench.torque.z = ft_data[5]

        dat_compliance_svr.header = make_header("tf_ft_front")
        dat_compliance_svr.wrench.force.x = svr_data[0]
        dat_compliance_svr.wrench.force.y = svr_data[1]
        dat_compliance_svr.wrench.torque.z = svr_data[2]

        dat_compliance_bumper_loc.data[0] = bumper_loc[0]
        dat_compliance_bumper_loc.data[1] = bumper_loc[1] * 180 / np.pi
        dat_compliance_bumper_loc.data[2] = bumper_loc[2]
        dat_compliance_bumper_loc.data[3] = bumper_loc[3]

        # rospy.loginfo(RosMassage)
        pub_emg.publish(FlagEmergency)
        pub_vel.publish(dat_vel)
        pub_twist.publish(qolo_twist)
        pub_cor_vel.publish(dat_cor_vel)
        pub_wheels.publish(dat_wheels)
        pub_user.publish(dat_user)
        pub_compliance_raw.publish(dat_compliance_raw)
        pub_compliance_svr.publish(dat_compliance_svr)
        pub_compliance_bumper_loc.publish(dat_compliance_bumper_loc)

        # rospy.loginfo(dat_user)
        rospy.loginfo(RosMassage)
        pub_mess.publish(RosMassage)
        rate.sleep()

    # Stop_Thread_Flag = True
    # thread_user.raise_exception()
    # thread_user.join()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass
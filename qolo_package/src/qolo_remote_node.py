#! /usr/bin/env python
#########  TEMPORAL Program only for HASLER Project ##########
##### Author: Diego F. Paez G.
##### Data: 2019/10/01
##### To be Deleted

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

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension 

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

REMOTE_MODE = True
SHARED_MODE = True
COMPLIANCE_FLAG = False
JOYSTICK_MODE = False

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
ZERO_LW = 2650 #2750;
ZERO_RW = 2500 #2650;
High_DAC = 5000;
MBED_Enable = mraa.Gpio(36) #11 17
MBED_Enable.dir(mraa.DIR_OUT)

GEAR = 12.64
DSITANCE_CW = 0.548/2  # DSITANCE_CW bettween two wheels
RADIUS = 0.304/2 # meter

MaxSpeed = 1.5 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
MinSpeed = MaxSpeed*backward_coefficient
MaxAngular = 4.124
W_ratio = 4 # Ratio of the maximum angular speed (232 deg/s)

Max_motor_v = (MaxSpeed/ (RADIUS*(2*np.pi))) *60*GEAR # max motor speed: 1200 rpm

# Setting for the RDS service
# Some reference for controlling the non-holonomic base
y_coordinate_of_reference_point_for_command_limits = 0.5;
# Gain to this point
weight_scaling_of_reference_point_for_command_limits = 0.;
# Some gain for velocity after proximity reaches limits
tau = 2.;
# Minimal distance to obstacles
delta = 0.08;
clearance_from_axle_of_final_reference_point = 0.15;
max_linear = MaxSpeed;
min_linear = -MinSpeed;
absolute_angular_at_min_linear = 0.;
absolute_angular_at_max_linear = 0.;
absolute_angular_at_zero_linear = MaxAngular/W_ratio;
linear_acceleration_limit = 1.1
angular_acceleration_limit = 1.5

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

# Parameters for compliant control
compliant_V =0.
compliant_W =0.

bumper_l = 0.2425      # (210+32.5) mm
bumper_R = 0.33 # 330 mm
Ts = 1.0/100    # 100 Hz
Damping_gain = 100           # 1 N-s/m 
robot_mass = 50         # 120 kg

# Global Variables for Compliant mode
offset_ft_data = np.zeros((6,))
raw_ft_data  = np.zeros((6,))
filtered_ft_data  = np.zeros((6,))
ft_data =  np.zeros((6,))
initialising_ft=True
init_ft_data = {
    'Fx': [],
    'Fy': [],
    'Fz': [],
    'Mx': [],
    'My': [],
    'Mz': [],
    }

# Prediction Models
bumperModel = None

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
        Remote_V = round(data.data[1],6)
        Remote_W =  round(data.data[2],6)
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
    global bumperModel
    # corr_ft_data = bumperModel.predict(ft_data)
    # ft_data_temp = ft_data
    corr_ft_data = bumperModel.predict(np.reshape(ft_data, (1,-1)))
    h = 0.1
    theta = 30 #np.pi/6
    return (corr_ft_data, h, theta)


def transform_to_bumper_surface(ft_data, h, theta):
    transformed_data = np.zeros((6,))

    # Fx
    transformed_data[0] = (ft_data[0] * math.cos(theta)
        - ft_data[1] * math.sin(theta))
    
    # Fy
    transformed_data[1] = ft_data[2]

    # Fz
    transformed_data[2] = (- ft_data[0] * math.sin(theta)
        - ft_data[1] * math.cos(theta))

    # Mx
    transformed_data[3] = (ft_data[3] * math.cos(theta)
        - ft_data[4] * math.sin(theta)
        - transformed_data[2] * h
        + transformed_data[1] * bumper_R)
    
    # My
    transformed_data[4] = (ft_data[5]
        - transformed_data[0] * bumper_R)

    # Mz
    transformed_data[5] = (- ft_data[3] * math.sin(theta)
        - ft_data[4] * math.cos(theta))
    
    return transformed_data


def compliance_control(v_prev, omega_prev, Fmag, h, theta):
    # F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x
    # And set reference to 0 and discretize w/ ZOH
    
    stheta = math.sin(theta)    # Small optimization
    ctheta = math.cos(theta)    # Small optimization

    # Position wrt center of rotatiion
    R = math.sqrt((bumper_R*stheta)**2 + (bumper_l + bumper_R*ctheta)**2 )
    beta = math.atan2(bumper_R * stheta, bumper_l)

    sbeta = math.sin(beta)      # Small optimization
    cbeta = math.cos(beta)      # Small optimization
    
    a = ctheta
    b = R*(stheta*cbeta - ctheta*sbeta)

    # Admittance Control
    v_eff_prev = (a * v_prev) + (b * omega_prev)

    v_eff_dot = (-Fmag - Damping_gain*v_eff_prev) / robot_mass
    v_eff = v_eff_dot * Ts + v_eff_prev

    # # Calculate new v and omega
    # c_prev = (-b * v_prev) + (a * omega_prev)
    # den = a**2 - b**2
    # v = (a*v_eff - b*c_prev) / den
    # omega = (-b*v_eff + a*c_prev) / den

    # Ensure non-zero 'a' and 'b'
    eps = 0.01
    if (a < eps):
        return (v_prev, v_eff/b)
    if (b < eps):
        return (v_eff/a, omega_prev)

    # Calculate new v and omega in parameterized form
    t = 0.5     # \in [0,1]
    v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
    omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev
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
    
    a = Xin[1]
    b = Xin[2]
    c = Xin[3]
    d = Xin[4]
    e = Xin[5]
    f = Xin[6]
    g = Xin[7]
    h = Xin[8]
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
    motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
    motor_w = (2*Max_motor_v/(DSITANCE_CW)*Command_W/5000 - Max_motor_v/(DSITANCE_CW)) / W_ratio # In [RPM]

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
    global DSITANCE_CW, RADIUS, User_V, User_W, MaxSpeed, GEAR, Max_motor_v

    # These lines should be commented to execute the RDS output
    # motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
    # motor_w = (2*Max_motor_v/(DSITANCE_CW)*Command_W/5000 - Max_motor_v/(DSITANCE_CW)) / W_ratio # In [RPM]
    # User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),4)
    # User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),4)

    # Using the desired velocity (linearn adn angular) --> transform to motor speed

    wheel_L = Desired_V - (DSITANCE_CW * Desired_W)    # Output in [m/s]
    wheel_R = Desired_V + (DSITANCE_CW * Desired_W)    # Output in [m/s]
    # print ('Wheels Vel =', wheel_L, wheel_R)

    # motor_v = round(((Desired_V*GEAR)/RADIUS)/(np.pi/30),8) 
    # motor_w = round(((Desired_W*GEAR)/RADIUS)/(np.pi/30),8) 

    # rpm_L = motor_v - DSITANCE_CW*motor_w
    # rpm_R = motor_v + DSITANCE_CW*motor_w
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

    # threadLock.acquire()
    conv.SET_DAC0(Send_DAC0, conv.data_format.voltage)
    conv.SET_DAC1(Send_DAC1, conv.data_format.voltage)
    conv.SET_DAC2(High_DAC, conv.data_format.voltage)
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
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()

    request.nominal_command.linear = User_V;
    request.nominal_command.angular = User_W;

    if cycle<=0.001:
        delta_time = 0.005;
    else:
        delta_time = time.clock() - cycle;

    response = RDS(request)
    Corrected_V = round(response.corrected_command.linear,6)
    Corrected_W = round(response.corrected_command.angular,6)

    cycle = time.clock()

# print cycle 
# except:
    # Output_V = User_V
    # Output_W = User_W
# print "RDS Service failed"

def mds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible, Corrected_V, Corrected_W
    # print "Waiting for MDS Service"

    rospy.wait_for_service('rds_velocity_command_correction')
    # try:
    RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

    request = VelocityCommandCorrectionRDSRequest()

    request.nominal_command.linear = User_V;
    request.nominal_command.angular = User_W;

    request.velocity_limits.max_linear = max_linear;
    request.velocity_limits.min_linear = min_linear;
    request.velocity_limits.abs_angular_at_min_linear = absolute_angular_at_min_linear;
    request.velocity_limits.abs_angular_at_max_linear = absolute_angular_at_max_linear;
    request.velocity_limits.abs_angular_at_zero_linear = absolute_angular_at_zero_linear;
    request.abs_linear_acceleration_limit = linear_acceleration_limit;
    request.abs_angular_acceleration_limit = angular_acceleration_limit;

    request.y_coordinate_of_reference_point_for_command_limits = y_coordinate_of_reference_point_for_command_limits;
    request.weight_scaling_of_reference_point_for_command_limits = weight_scaling_of_reference_point_for_command_limits;
    request.clearance_from_axle_of_final_reference_point = clearance_from_axle_of_final_reference_point;
    request.delta = delta;
    request.tau = tau;
    request.y_coordinate_of_reference_biasing_point = 1.;
    request.weight_of_reference_biasing_point = 0.;

    request.last_actual_command.linear = last_v;
    request.last_actual_command.angular = last_w;

    if cycle==0:
        delta_time = 0.005;
    else:
        delta_time = time.clock() - cycle;

    request.command_cycle_time = delta_time
    request.abs_linear_acceleration_limit = 4;
    request.abs_angular_acceleration_limit = 2;

    response = RDS(request)
    Corrected_V = round(response.corrected_command.linear,6)
    Corrected_W = round(response.corrected_command.angular,6)
    feasible = response.feasible

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
    global counter1, compliant_V, compliant_W, raw_ft_data, ft_data, offset_ft_data
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
        motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
        motor_w = (2*Max_motor_v/(DSITANCE_CW)*Command_W/5000 - Max_motor_v/(DSITANCE_CW)) / W_ratio # In [RPM]
        User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),6)
        User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),6)


    if FLAG_debug:
        FSR_time = round((time.clock() - t1),4)
        t1 = time.clock()

    if SHARED_MODE:
        rds_service()
    else:
        Corrected_V = User_V
        Corrected_W = User_W

    if COMPLIANCE_FLAG:
        [ft_data, h, theta] = damper_correction(raw_ft_data - offset_ft_data)
        ft_data = transform_to_bumper_surface(ft_data, h, theta)
        [compliant_V, compliant_W] = compliance_control(Corrected_V, Corrected_W, ft_data[2], h, theta)
        Output_V = round(compliant_V,6)
        Output_W = round(compliant_W,6)
    else:
        Output_V = Corrected_V
        Output_W = Corrected_W


    if FLAG_debug:
        RDS_time = round((time.clock() - t1),4)
        t1 = time.clock()
    
    # Debugging the speed controller
    # if counter1 < 20:
    #     Comand_DAC0 = 4000
    #     Comand_DAC1 = 4000
    # else:
    #     Comand_DAC0 = ZERO_LW
    #     Comand_DAC1 = ZERO_RW

    if Output_V > MaxSpeed:
        Output_V = MaxSpeed
    elif Output_V < -MinSpeed:
        Output_V = -MinSpeed

    if Output_W > MaxAngular:
        Output_W = MaxAngular
    elif Output_W < -MaxAngular:
        Output_W = -MaxAngular

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
    global compliant_V, compliant_W, offset_ft_data, bumperModel
    prevT = 0
    FlagEmergency=False
    # threadLock = threading.Lock()
    # Setting ROS Node
    
    # Call the calibration File
    # load_calibration()
    if COMPLIANCE_FLAG:
        print("loading SVR models")
        bumperModel = BumperModel()

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
    dat_user = Float32MultiArray()
    dat_user.layout.dim.append(MultiArrayDimension())
    dat_user.layout.dim[0].label = 'FSR_read'
    dat_user.layout.dim[0].size = 11
    dat_user.data = [0]*11

    qolo_twist = Twist()
    qolo_twist.linear.x = 0
    qolo_twist.linear.y = 0
    qolo_twist.linear.z = 0
    qolo_twist.angular.x = 0
    qolo_twist.angular.y = 0
    qolo_twist.angular.z = 0
    dat_vel = Float32MultiArray()
    dat_vel.layout.dim.append(MultiArrayDimension())
    dat_vel.layout.dim[0].label = 'Velocities: last message, Input[2], Output[2]'
    dat_vel.layout.dim[0].size = 5
    dat_vel.data = [0]*4

    dat_cor_vel = Float32MultiArray()
    dat_cor_vel.layout.dim.append(MultiArrayDimension())
    dat_cor_vel.layout.dim[0].label = 'Velocities: User[2], Corrected_OA[2], Corrected_Compliance[2]'
    dat_cor_vel.layout.dim[0].size = 6
    dat_cor_vel.data = [0]*6

    dat_wheels = Float32MultiArray()
    dat_wheels.layout.dim.append(MultiArrayDimension())
    dat_wheels.layout.dim[0].label = 'Wheels Output'
    dat_wheels.layout.dim[0].size = 2
    dat_wheels.data = [0]*2

    dat_compliance = Wrench()

    pub_wheels = rospy.Publisher('qolo/wheels', Float32MultiArray, queue_size=1)
    pub_twist = rospy.Publisher('qolo/twist', Twist, queue_size=1)
    pub_vel = rospy.Publisher('qolo/velocity', Float32MultiArray, queue_size=1)
    pub_cor_vel = rospy.Publisher('qolo/corrected_velocity', Float32MultiArray, queue_size=1)
    pub_emg = rospy.Publisher('qolo/emergency', Bool, queue_size=1)
    pub_user = rospy.Publisher('qolo/user_input', Float32MultiArray, queue_size=1)
    pub_compliance = rospy.Publisher('qolo/compliance', Wrench, queue_size=10)
    
    pub_mess = rospy.Publisher('qolo/message', String, queue_size=1)
    rospy.init_node('qolo_remote', anonymous=True)
    rate = rospy.Rate(100) #  100 hz


    if COMPLIANCE_FLAG:
        ftsub = rospy.Subscriber("/rokubi_node_front/ft_sensor_measurements",WrenchStamped,ft_sensor_callback)
        start_time = time.time()
        initialising_ft = True
        print('Waiting for FT Sensor Offset: 4 sec')
        # while (time.time() - start_time > 4): # 4 s for initialising ft sensor
        time.sleep(4)
        initialising_ft = False
        offset_ft_data = np.array([
            np.mean(init_ft_data['Fx']),
            np.mean(init_ft_data['Fy']),
            np.mean(init_ft_data['Fz']),
            np.mean(init_ft_data['Mx']),
            np.mean(init_ft_data['My']),
            np.mean(init_ft_data['Mz']),
        ])
        print('Starting Compliant Mode')
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
        RosMassage = "%s %s %s %s %s %s" % (User_V, User_W, compliant_V, compliant_W, Output_V, Output_W)
        dat_wheels.data = [Send_DAC0, Send_DAC1]
        dat_vel.data = [time_msg, User_V, User_W, Output_V, Output_W]
        qolo_twist.linear.x = Output_V
        qolo_twist.angular.z = Output_W
        dat_cor_vel.data = [User_V, User_W, Corrected_V, Corrected_W, compliant_V, compliant_W]
        dat_user.data = [Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9],Out_CP]
        

        #### CHANGE THIS TO WRENCH TYPE ##############
        dat_compliance.force.x = ft_data[0]; dat_compliance.force.y = ft_data[1]; dat_compliance.force.z = ft_data[2]
        dat_compliance.torque.x = ft_data[3]; dat_compliance.torque.y = ft_data[4]; dat_compliance.torque.z = ft_data[5]

        # rospy.loginfo(RosMassage)
        pub_emg.publish(FlagEmergency)
        pub_vel.publish(dat_vel)
        pub_twist.publish(qolo_twist)
        pub_cor_vel.publish(dat_cor_vel)
        pub_wheels.publish(dat_wheels)
        pub_user.publish(dat_user)
        pub_compliance.publish(dat_compliance)

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
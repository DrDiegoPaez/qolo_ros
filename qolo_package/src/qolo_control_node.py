#! /usr/bin/env python

import HighPrecision_ADDA_Double as converter
import time
import math
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

import rospy
from std_msgs.msg import String
import threading

from rds_network_ros.srv import *

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
ZERO_V = 2590;
High_DAC = 5000;
MBED_Enable = mraa.Gpio(36) #11 17
MBED_Enable.dir(mraa.DIR_OUT)


GEAR = 12.64
DISTANCE = 0.62/2  # distance bettween two wheels
RADIUS = 0.304/2 # meter

MaxSpeed = 1.0 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
MinSpeed = MaxSpeed*backward_coefficient
MaxAngular = 4.124
W_ratio = 3 # Ratio of the maximum angular speed (232 deg/s)

Max_motor_v = (MaxSpeed/ (RADIUS*(2*np.pi))) *60*GEAR # max motor speed: 1200 rpm

# Setting for the RDS service
max_linear = MaxSpeed;
min_linear = -MinSpeed;
absolute_angular_at_min_linear = 0.;
absolute_angular_at_max_linear = 0.;
absolute_angular_at_zero_linear = MaxAngular/W_ratio;
linear_acceleration_limit = 1.
angular_acceleration_limit = 1.5
feasible = 0
Output_V = 0.;
Output_W = 0.;
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
FsrZero = np.array([100.1, 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11, 200.1])
# default value for pre-configuration
# k1, k2, k3, k4, k5, k6, k7, k8 =    0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 0.57, 0.63 # 2.48, 0.91, 1.59, 1.75, 1.46
# FsrK = np.array([0.63, 0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 0.57, 0.63, 0.63])
FsrK = np.array([0.8, 0.8, 1.0, 0.8, 0.5, 0.5, 0.8, 1.0, 0.8, 0.8])

# Vector input for all sensor data
# Xin = np.zeros((10))
Xin = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
Xin_temp = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
ComError = 0
RemoteE = 0

# # coefficient for calculate center of pressure: ox
Rcenter = np.array([0., -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 0.])

# classification point for center of pressure ox(calibration needed)
pl2, pl1, pr1, pr2 = -1.9, -0.7, 0.7, 1.9
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

# read data from ADDA board
def read_FSR():
    global Xin_temp, RemoteE, ComError
    # Checking emergency inputs
    RemoteE = conv.ReadChannel(7, conv.data_format.voltage)
    ComError = conv.ReadChannel(6, conv.data_format.voltage)
    Xin_temp[0] = float(conv.ReadChannel(5, conv.data_format.voltage))
    Xin_temp[1] = float(conv.ReadChannel(15, conv.data_format.voltage))
    Xin_temp[2] = float(conv.ReadChannel(14, conv.data_format.voltage))
    Xin_temp[3] = float(conv.ReadChannel(13, conv.data_format.voltage))
    Xin_temp[4] = float(conv.ReadChannel(12, conv.data_format.voltage))
    Xin_temp[5] = float(conv.ReadChannel(11, conv.data_format.voltage))
    Xin_temp[6] = float(conv.ReadChannel(10, conv.data_format.voltage))
    Xin_temp[7] = float(conv.ReadChannel(9, conv.data_format.voltage))
    Xin_temp[8] = float(conv.ReadChannel(8, conv.data_format.voltage))
    Xin_temp[9] = float(conv.ReadChannel(4, conv.data_format.voltage))

    # return Xin_temp
# execution command to DAC board based on the output curve
def execution():

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
    
    forward, backward, left_angle_for, left_angle_turn, right_angle_for, right_angle_turn, left_around, right_around = output(a, b, c, d, e, f, g, h, ox)

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
    Xin = FsrK* (Xin_temp - FsrZero)     # Values in [mV]

    # Calculating the Center of pressure
    ox = np.sum(Rcenter*Xin) / (Xin[1] + Xin[2] + Xin[3] + Xin[4] + Xin[5] + Xin[6] + Xin[7] + Xin[8])
    Out_CP = round(ox, 4);
    # Runs the user input and returns Command_V and Command_W --> in 0-5k scale
    execution()
    motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
    motor_w = (2*Max_motor_v/(DISTANCE)*Command_W/5000 - Max_motor_v/(DISTANCE)) / W_ratio # In [RPM]

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
    conv.SET_DAC0(ZERO_V, conv.data_format.voltage)
    conv.SET_DAC1(ZERO_V, conv.data_format.voltage)

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

# for interruption
def exit():
    global Stop_Thread_Flag
    # MBED_Enable = mraa.Gpio(36) #11 17
    # MBED_Enable.dir(mraa.DIR_OUT)
    MBED_Enable.write(0)
    conv.SET_DAC2(0, conv.data_format.voltage)
    conv.SET_DAC3(0, conv.data_format.voltage)
    conv.SET_DAC0(0, conv.data_format.voltage)
    conv.SET_DAC1(0, conv.data_format.voltage)
    Stop_Thread_Flag = True
    # cleanup_stop_thread()

    print('----> You choose to interrupt')
    exit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

def transformTo_Lowevel(Desired_V, Desired_W):
    # print('received ', Command_V, Command_W)
    global DISTANCE, RADIUS, User_V, User_W, MaxSpeed, GEAR, Max_motor_v

    # These lines should be commented to execute the RDS output
    # motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
    # motor_w = (2*Max_motor_v/(DISTANCE)*Command_W/5000 - Max_motor_v/(DISTANCE)) / W_ratio # In [RPM]
    # User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),4)
    # User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),4)

    # Using the returned velocity from the SRD constraints
    motor_v = round(((Desired_V*GEAR)/RADIUS)/(np.pi/30),4) 
    motor_w = round(((Desired_W*GEAR)/RADIUS)/(np.pi/30),4) 

    # print("left wheel = ",motor_v, "right wheel = ",motor_w)
    rpm_L = motor_v - DISTANCE*motor_w
    rpm_R = motor_v + DISTANCE*motor_w
    
    # User_V = round( ((rpm_R+rpm_L)*RADIUS*(np.pi/60)), 4)
    # User_W = round( (((rpm_R-rpm_L)*6)/DISTANCE), 4)

    # print("left wheel = ",rpm_L, "right wheel = ",rpm_R)
    Command_L = 5000*rpm_L/2400 + ZERO_V
    Command_R = 5000*rpm_R/2400 + ZERO_V
    # print('transformed ', Command_L, Command_R)
    Command_L = round(Command_L, 4)
    Command_R = round(Command_R, 4)
    # print('transformed ', Command_L, Command_R)
    return Command_L, Command_R



# output curve: Linear/Angular Velocity-Pressure Center
def output(a, b, c, d, e, f, g, h, ox):
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

def write_DA():
    global Comand_DAC0, Comand_DAC1, Send_DAC0, Send_DAC1, Command_V, Command_W
    Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Output_V, Output_W)
    # Error in the ADC Board connection
    if Comand_DAC0 < 4870:
        Send_DAC0 = Comand_DAC0 +130
    else:
        Send_DAC0 = 5000
    if Comand_DAC0 < 4870:
        Send_DAC1 = Comand_DAC1
    else:
        Send_DAC1 = 4870
    # threadLock.acquire()
    conv.SET_DAC0(Send_DAC0, conv.data_format.voltage)
    conv.SET_DAC1(Send_DAC1, conv.data_format.voltage)
    conv.SET_DAC2(High_DAC, conv.data_format.voltage)
    # threadLock.release()

def rds_service():
    global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible
    # print "Waiting for RDS Service"

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

    request.last_actual_command.linear = last_v;
    request.last_actual_command.angular = last_w;
    request.command_cycle_time = time.clock() - cycle;
    request.abs_linear_acceleration_limit = 4;
    request.abs_angular_acceleration_limit = 2;

    response = RDS(request)
    Output_V = round(response.corrected_command.linear,4)
    Output_W = round(response.corrected_command.angular,4)
    feasible = response.feasible

    last_v = Output_V
    last_w = Output_W
    cycle = time.clock()
# print cycle 
# except:
    # Output_V = User_V
    # Output_W = User_W
# print "RDS Service failed"


def control():
    global A1, B1, C1, D1, E1, F1, G1, H1
    # global r1, r2, r3, r4, r5, r6, r7, r8
    global Rcenter
    global Command_V, Command_W, Comand_DAC0, Comand_DAC1, User_V, User_W, Output_V, Output_W
    global counter1
    global DA_time, RDS_time, Compute_time, FSR_time
    
    # Replace with a node subsription
    global Xin, FsrZero, FsrK, Out_CP
    if FLAG_debug:
        t1 = time.clock()

    read_FSR()

    if FLAG_debug:
        FSR_time = round((time.clock() - t1),4)
        t1 = time.clock()
    # FSR Inputs calibration: 
    Xin = FsrK* (Xin_temp - FsrZero)     # Values in [mV]
    # Calculating the Center of pressure
    ox = np.sum(Rcenter*Xin) / (Xin[1] + Xin[2] + Xin[3] + Xin[4] + Xin[5] + Xin[6] + Xin[7] + Xin[8])
    Out_CP = round(ox, 4);
    execution()  # Runs the user input with Out_CP and returns Command_V and Command_W --> in 0-5k scale
    motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
    motor_w = (2*Max_motor_v/(DISTANCE)*Command_W/5000 - Max_motor_v/(DISTANCE)) / W_ratio # In [RPM]
    User_V = round(((motor_v/GEAR)*RADIUS)*(np.pi/30),4)
    User_W = round(((motor_w/GEAR)*RADIUS)*(np.pi/30),4)
    
    if FLAG_debug:
        t1 = time.clock()
    
    rds_service()
    
    # Output_V = User_V
    # Output_W = User_W
    if FLAG_debug:
        RDS_time = round((time.clock() - t1),4)

    # Debugging the speed controller
    # if counter1 < 20:
    #     Comand_DAC0 = 4000
    #     Comand_DAC1 = 4000
    # else:
    #     Comand_DAC0 = ZERO_V
    #     Comand_DAC1 = ZERO_V
    if FLAG_debug:
        t1 = time.clock()
    write_DA()
    if FLAG_debug:
        DA_time = round((time.clock() - t1),4)
    # print ('FSR_read: %s, FSR_read: %s, FSR_read: %s, FSR_read: %s,')

    counter1 += 1  # for estiamting frequency

def control_node():
    global Comand_DAC0, Comand_DAC1, Send_DAC0, Send_DAC1
    global RemoteE, ComError
    global DA_time, RDS_time, Compute_time, FSR_time, extra_time
    prevT = 0
    # threadLock = threading.Lock()
    # Setting ROS Node
    
    # Call the calibration File
    # load_calibration()
    

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
    pub = rospy.Publisher('qolo', String, queue_size=1)
    rospy.init_node('qolo_control', anonymous=True)
    rate = rospy.Rate(50) #  20 hz

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
            FlagEmergency=1
            # threadLock.acquire()
            while FlagEmergency:
                conv.SET_DAC2(0, conv.data_format.voltage)
                conv.SET_DAC0(ZERO_V, conv.data_format.voltage)
                conv.SET_DAC1(ZERO_V, conv.data_format.voltage)
                ResetFSR = conv.ReadChannel(5, conv.data_format.voltage)
                if ResetFSR >= THRESHOLD_V:
                    print('ResetFSR ', ResetFSR)
                    FlagEmergency=0
                    enable_mbed()
                time.sleep(0.1)
                # threadLock.release()

        cycle_T = time.clock() - prevT
        prevT = time.clock()
        now = datetime.datetime.now()
        current_time = now.strftime("%H:%M:%S")
        RosMassage = "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s" % (current_time, cycle_T, RDS_time,Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9],Out_CP,Send_DAC0, Send_DAC1, User_V, User_W, feasible, Output_V, Output_W)
        # RosMassage = "%s %s %s %s %s %s %s %s" % (cycle_T, RDS_time, DA_time, feasible, User_V, User_W, round(Output_V,4), round(Output_W,4) )
        # RosMassage = "%s %s %s %s %s" % (User_V, User_W, Output_V, Output_W, feasible)
        rospy.loginfo(RosMassage)
        pub.publish(RosMassage)
        rate.sleep()

    # Stop_Thread_Flag = True
    # thread_user.raise_exception()
    # thread_user.join()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

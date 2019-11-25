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

import rospy
cd srcfrom std_msgs.msg import String


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


# real zero point of each sensor
a_zero, b_zero, c_zero, d_zero, e_zero, f_zero, g_zero, h_zero = 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11

# FsrZero = arr.array('d',[200.1 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11 200.1])
FsrZero = np.array([100.1, 305.17, 264.7, 441.57, 336.46, 205.11, 441.57, 336.46, 205.11, 200.1])
# default value for pre-configuration
# k1, k2, k3, k4, k5, k6, k7, k8 =    0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 0.57, 0.63 # 2.48, 0.91, 1.59, 1.75, 1.46
FsrK = np.array([0.63, 0.63, 1.04, 0.8, 0.57, 0.63, 0.8, 0.57, 0.63, 0.63])

# Vector input for all sensor data
# Xin = np.zeros((10))
Xin = np.array([0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.])

# # coefficient for calculate center of pressure: ox
Rcenter = np.array([0., -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 0.])

# classification point for center of pressure ox(calibration needed)
pl2, pl1, pr1, pr2 = -1.9, -0.6, 0.6, 1.9
# -1.72, 0.075, 1.455, 1.98 
# -2.42, 0.67, 1.27, 1.82  
# -0.97, -0.2, 0.2, 1.17

# read k* and p* from txt file
# f1 = open('k*+p*write.txt')
# lines = f1.readlines()
# K = lines[0].strip('\n')
# K = K.split(',')
# P = lines[1].strip('\n')
# P = P.split(',')
# k1, k2, k3, k4, k5 = float(K[0]),float(K[1]),float(K[2]),float(K[3]),float(K[4])
# pl2, pl1, pr1, pr2 = float(P[0]),float(P[1]),float(P[2]),float(P[3])
# print(k1, k2, k3, k4, k5)
# print(pl2, pl1, pr1, pr2 )
# f1.close()

GEAR = 12.64
DISTANCE = 0.62/2  # distance bettween two wheels
RADIUS = 0.304/2 # meter

MaxSpeed = 1.0 # max Qolo speed: 1.51 m/s               --> Equivalent to 5.44 km/h
W_ratio = 2 # Ratio of the maximum angular speed (232 deg/s)

Max_motor_v = (MaxSpeed/ (RADIUS*(2*np.pi))) *60*GEAR # max motor speed: 1200 rpm

Command_V = 2500
Command_W = 2500
Comand_DAC0 = 0
Comand_DAC1 = 0
rpm_L = 0;
rpm_R = 0;

# Global variables for logging
Out_v = 0;
Out_w = 0;
Out_CP = 0;
Out_F = 0;

counter1 = 0
number = 100

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

def enable_mbed():
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

# for interruption
def exit(signum, frame):
    # MBED_Enable = mraa.Gpio(36) #11 17
    # MBED_Enable.dir(mraa.DIR_OUT)
    MBED_Enable.write(0)
    conv.SET_DAC2(0, conv.data_format.voltage)
    conv.SET_DAC3(0, conv.data_format.voltage)
    conv.SET_DAC0(0, conv.data_format.voltage)
    conv.SET_DAC1(0, conv.data_format.voltage)
    print('----> You choose to interrupt')
    exit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

def transformTo_Lowevel(Command_V, Command_W):
    # print('received ', Command_V, Command_W)
    global DISTANCE, RADIUS, Out_v, Out_w, MaxSpeed, GEAR, Max_motor_v, rpm_L, rpm_R

    motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v            # In [RPM]
    motor_w = (2*Max_motor_v/(DISTANCE)*Command_W/5000 - Max_motor_v/(DISTANCE)) / W_ratio # In [RPM]

    # Out_v = round((motor_v*RADIUS)*(np.pi/30),4)
    # Out_w = round((motor_w*6),4)

    # print("left wheel = ",motor_v, "right wheel = ",motor_w)
    rpm_L = motor_v - DISTANCE*motor_w
    rpm_R = motor_v + DISTANCE*motor_w
    
    Out_v = round( ((rpm_R+rpm_L)*RADIUS*(np.pi/60)), 4)
    Out_w = round( (((rpm_R-rpm_L)*6)/DISTANCE), 4)

    # print("left wheel = ",rpm_L, "right wheel = ",rpm_R)
    Command_L = 5000*rpm_L/2400 + ZERO_V
    Command_R = 5000*rpm_R/2400 + ZERO_V
    # print('transformed ', Command_L, Command_R)
    Command_L = round(Command_L, 4)
    Command_R = round(Command_R, 4)
    # print('transformed ', Command_L, Command_R)
    return Command_L, Command_R

# def transformTo_Lowevel(Command_V, Command_W):
#     global DISTANCE, RADIUS
#     # Command_W = 5000
#     MaxSpeed = 5.44 # max Qolo speed: km/h
#     Max_motor_v = MaxSpeed*1000/3600/RADIUS/(2*np.pi)*60*GEAR # max motor speed: 1200 rpm
#     motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v
#     motor_w = (2*Max_motor_v/(DISTANCE/2)*Command_W/5000 - Max_motor_v/(DISTANCE/2)) / W_ratio
#     # print("left wheel = ",motor_v, "right wheel = ",motor_w)
#     rpm_L = motor_v - DISTANCE*motor_w/2
#     rpm_R = motor_v + DISTANCE*motor_w/2
#     # print("left wheel = ",rpm_L, "right wheel = ",rpm_R)
#     Command_L = 5000*rpm_L/2400 + 2500
#     Command_R = 5000*rpm_R/2400 + 2500
#     Command_L = round(Command_L, 2)
#     Command_R = round(Command_R, 2)
#     # print(Command_L, Command_R)
#     return Command_L, Command_R

def log(a0, b0, c0, d0, e0, f0, g0, h0, a, b, c, d, e, f, g, h, ox, Command_V, Command_W, Comand_DAC0, Comand_DAC1, filename,level,fmt='%(created).6f %(message)s'):
    logger = logging.getLogger(filename)
    # logger.propagate = False
    format_str = logging.Formatter(fmt)
    logger.setLevel(level_relations.get(level))
    sh = logging.StreamHandler()
    sh.setFormatter(format_str)
    th = logging.handlers.TimedRotatingFileHandler(filename=filename,encoding='utf-8')
    th.setFormatter(format_str)
    logger.addHandler(sh)
    logger.addHandler(th)
    # logger.debug("%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s", a0, b0, c0, d0, e0, f0, g0, h0, a, b, c, d, e, f, g, h, ox, Command_V, Command_W, Comand_DAC0, Comand_DAC1)
    # logger.info("")
    logger.info("%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s", a0, b0, c0, d0, e0, f0, g0, h0, a, b, c, d, e, f, g, h, ox, Command_V, Command_W, Comand_DAC0, Comand_DAC1)
    logger.removeHandler(sh)
    logger.removeHandler(th)

# read data from ADDA board
def read_FSR():
    Xin[0] = float(conv.ReadChannel(5, conv.data_format.voltage))
    Xin[1] = float(conv.ReadChannel(15, conv.data_format.voltage))
    Xin[2] = float(conv.ReadChannel(14, conv.data_format.voltage))
    Xin[3] = float(conv.ReadChannel(13, conv.data_format.voltage))
    Xin[4] = float(conv.ReadChannel(12, conv.data_format.voltage))
    Xin[5] = float(conv.ReadChannel(11, conv.data_format.voltage))
    Xin[6] = float(conv.ReadChannel(10, conv.data_format.voltage))
    Xin[7] = float(conv.ReadChannel(9, conv.data_format.voltage))
    Xin[8] = float(conv.ReadChannel(8, conv.data_format.voltage))
    Xin[9] = float(conv.ReadChannel(4, conv.data_format.voltage))


def zeroCalibration():
    global CZero
    global Xin

    Xin = Xin - FsrZero

# pre_calibration with default value
def pre_calibration():
    # global k0, k1, k2, k3, k4, k5, k6, k7, k8, k9
    global FsrK
    global Xin

    Xin = FsrK * Xin

def collect():
    counter = 0
    while counter < number:
        read()

        AA.append(a)
        BB.append(b)
        CC.append(c)
        DD.append(d)
        EE.append(e)
        FF.append(c)
        GG.append(d)
        HH.append(e)

        conv.SET_DAC1(4500, conv.data_format.voltage)

        counter += 1
        time.sleep(0.01)

    conv.SET_DAC1(2500, conv.data_format.voltage)
    time.sleep(2)

def pos_deal():
    global k1, k2, k3, k4, k5, k6, k7, k8
    global pl2, pl1, pr1, pr2
    # pre-calibration
    AA1 = heapq.nlargest(len(AA) / 10, AA)
    BB1 = heapq.nlargest(len(BB) / 10, BB)
    CC1 = heapq.nlargest(len(CC) / 10, CC)
    DD1 = heapq.nlargest(len(DD) / 10, DD)
    EE1 = heapq.nlargest(len(EE) / 10, EE)
    FF1 = heapq.nlargest(len(FF) / 10, FF)
    GG1 = heapq.nlargest(len(GG) / 10, GG)
    HH1 = heapq.nlargest(len(HH) / 10, HH)

    aa = np.mean(AA1)
    bb = np.mean(BB1)
    cc = np.mean(CC1)
    dd = np.mean(DD1)
    ee = np.mean(EE1)
    ff = np.mean(FF1)
    gg = np.mean(GG1)
    hh = np.mean(HH1)

    print("AA1 = ", AA1)
    print("----------------")
    print("BB1 = ", BB1)
    print("----------------")
    print("CC1 = ", CC1)
    print("----------------")
    print("DD1 = ", DD1)
    print("----------------")
    print("EE1 = ", EE1)
    print("----------------")
    print("FF1 = ", FF1)
    print("----------------")
    print("GG1 = ", GG1)
    print("----------------")
    print("HH1 = ", HH1)
    print("----------------")

    print("max_average = ", aa, bb, cc, dd, ee, ff, gg, hh)
    k1 = round((4500 - 2500) / aa, 2)
    k2 = round((4500 - 2500) / bb, 2)
    k3 = round((4500 - 2500) / cc, 2)
    k4 = round((4500 - 2500) / dd, 2)
    k5 = round((4500 - 2500) / ee, 2)
    print("coefficient = ", k1, k2, k3, k4, k5)

    # post-calibration
    AA2 = [AA[i] * k1 for i in range(len(AA))]
    BB2 = [BB[i] * k2 for i in range(len(AA))]
    CC2 = [CC[i] * k3 for i in range(len(AA))]
    DD2 = [DD[i] * k4 for i in range(len(AA))]
    EE2 = [EE[i] * k5 for i in range(len(AA))]

    ox = [(-2 * AA2[i] - BB2[i] + 0 * CC2[i] + DD2[i] + 2 * EE2[i]) / (AA2[i] + BB2[i] + CC2[i] + DD2[i] + EE2[i]) for i
          in range(len(AA))]

    ox_l2 = (np.mean(ox[0:number]) + np.mean(ox[9 * number:10 * number])) / 2
    ox_l1 = (np.mean(ox[number:2 * number]) + np.mean(ox[8 * number:9 * number])) / 2
    ox_0 = (np.mean(ox[2 * number:3 * number]) + np.mean(ox[7 * number:8 * number])) / 2
    ox_r1 = (np.mean(ox[3 * number:4 * number]) + np.mean(ox[6 * number:7 * number])) / 2
    ox_r2 = (np.mean(ox[4 * number:5 * number]) + np.mean(ox[5 * number:6 * number])) / 2
    ox_l2 = round(ox_l2, 2)
    ox_l1 = round(ox_l1, 2)
    ox_0 = round(ox_0, 2)
    ox_r1 = round(ox_r1, 2)
    ox_r2 = round(ox_r2, 2)

    pl2, pl1, pr1, pr2 = ox_l2 + 0.2, ox_0 - 0.3, ox_0 + 0.3, ox_r2 - 0.2
    print("ox_* = ", ox_l2, ox_l1, ox_0, ox_r1, ox_r2)
    print("p_*0 = ", ox_l2 + 0.2, ox_0 - 0.3, ox_0 + 0.3, ox_r2 - 0.2)
    print("p_*1 = ", (ox_l2 + ox_l1)/2, (ox_l1 + ox_0)/2, (ox_0 + ox_r1)/2, (ox_r1 + ox_r2)/2)

    f0 = open('k*+p*add.txt', 'a')
    f0.writelines(str(k1) + ',' + str(k2) + ',' + str(k3) + ',' + str(k4) + ',' + str(k5))
    f0.write('\n')
    f0.writelines(str(pl2) + ',' + str(pl1) + ',' + str(pr1) + ',' + str(pr2))
    f0.write('\n')
    f0.close()
    f1 = open('k*+p*write.txt', 'w')
    f1.writelines(str(k1) + ',' + str(k2) + ',' + str(k3) + ',' + str(k4) + ',' + str(k5))
    f1.write('\n')
    f1.writelines(str(pl2) + ',' + str(pl1) + ',' + str(pr1) + ',' + str(pr2))
    f1.write('\n')
    f1.close()

# online calibration
def calibration():
    AA = []
    BB = []
    CC = []
    DD = []
    EE = []
    conv.SET_DAC1(2500, conv.data_format.voltage)
    time.sleep(3)
    counter0 = 0
    while counter0 < 10:
        collect()
        counter0 += 1
    pos_deal()
    print 'online calibration finished'
    time.sleep(3)

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

    treshold  = 800 # avoid unstoppable and undistinguishable
    
    forward, backward, left_angle_for, left_angle_turn, right_angle_for, right_angle_turn, left_around, right_around = output(a, b, c, d, e, f, g, h, ox)
    # forward = round(forward, 0)
    # backward = round(backward, 0)
    # left_angle_for = round(left_angle_for, 0)
    # left_angle_turn = round(left_angle_turn, 0)
    # right_angle_for = round(right_angle_for, 0)
    # right_angle_turn = round(right_angle_turn, 0)
    # left_around = round(left_around, 0)
    # right_around = round(right_around, 0)

    if ox <= pr1 and ox >= pl1 and d >= treshold and e >= treshold:
        Command_V = forward
        Command_W = 2500
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)

        # continue
    # turn an angle
    elif (ox <= pl1 and ox >= pl2) and h <= treshold and (c >= treshold or b >= treshold):
        Command_V = left_angle_for
        Command_W = left_angle_turn
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)

        # continue
    elif (ox >= pr1 and ox <= pr2) and a <= treshold and (f >= treshold or g >= treshold):
        Command_V = right_angle_for
        Command_W = right_angle_turn
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)

        # continue
    # turn around
    elif ox <= pl2 and h <= treshold and (a >= treshold or b >= treshold):
        Command_V = 2500
        Command_W = left_around
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)

        # continue
    elif ox >= pr2 and a<=treshold and (g >= treshold or h >= treshold):
        Command_V = 2500
        Command_W = right_around
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)
        
    # backward
    elif a >= treshold and h >= treshold and d < 300 and e < 300:
        Command_V = backward
        Command_W = 2500
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)
        
    else:
        Command_V = 2500
        Command_W = 2500
        Comand_DAC0, Comand_DAC1 = transformTo_Lowevel(Command_V, Command_W)
        


def write_DA():
        conv.SET_DAC0(Comand_DAC0, conv.data_format.voltage)
        conv.SET_DAC1(Comand_DAC1, conv.data_format.voltage)
        conv.SET_DAC2(High_DAC, conv.data_format.voltage)

def control():
    global A1, B1, C1, D1, E1, F1, G1, H1
    # global r1, r2, r3, r4, r5, r6, r7, r8
    global Rcenter
    global Command_V, Command_W, Comand_DAC0, Comand_DAC1
    global counter1
    global Xin, FsrZero, FsrK, Out_CP
    
    read_FSR()

    if Xin[0] > THRESHOLD_V and Xin[10] > THRESHOLD_V:
         calibration()

    # FSR Inputs: 
    Xin = Xin - FsrZero     # Values in [mV]
    Xin = FsrK * Xin        # Values in [mV]

    # Calculating the Center of pressure
    ox = np.sum(Rcenter*Xin) / (Xin[1] + Xin[2] + Xin[3] + Xin[4] + Xin[5] + Xin[6] + Xin[7] + Xin[8])

    ox = round(ox, 4)   # Round to .4 decimals
    Out_CP = ox;
    # print ox
    execution()

    write_DA()

    counter1 += 1  # for estiamting frequency

    # log file
    # a0, b0, c0, d0, e0, f0, g0, h0 = str(a0),str(b0),str(c0),str(d0),str(e0),str(f0),str(g0),str(h0)
    # a0, b0, c0, d0, e0, f0, g0, h0 = a0.ljust(8),b0.ljust(8),c0.ljust(8),d0.ljust(8),e0.ljust(8),f0.ljust(8),g0.ljust(8),h0.ljust(8)
    # a, b, c, d, e, f, g, h, ox, = str(a),str(b),str(c),str(d),str(e),str(f),str(g),str(h),str(ox)
    # a, b, c, d, e, f, g, h, ox, = a.ljust(8),b.ljust(8),c.ljust(8),d.ljust(8),e.ljust(8),f.ljust(8),g.ljust(8),h.ljust(8),ox.ljust(8)
    # Command_V, Command_W = str(Command_V),str(Command_W)
    # Command_V, Command_W = Command_V.ljust(8), Command_W.ljust(8)
    # Comand_DAC0, Comand_DAC1 = str(Comand_DAC0),str(Comand_DAC1)
    # Comand_DAC0, Comand_DAC1 = Comand_DAC0.ljust(8), Comand_DAC1.ljust(8)

    

    # RosMassage = ("%s %s %s %s %s", ox, Command_V, Command_W, Comand_DAC0, Comand_DAC1)
    # return RosMassage

    # log(a0, b0, c0, d0, e0, f0, g0, h0, a, b, c, d, e, f, g, h, ox, Command_V, Command_W, Comand_DAC0, Comand_DAC1, '/home/qolo/catkin_ws/src/qoloT_control/logs/qolot.log', level='info')

    # time.sleep(0.01)

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)


# start = time.time()  # for calculate frequency
# try:

#     enable_mbed()
#     while True:
#         control()
#         RemoteE = conv.ReadChannel(7, conv.data_format.voltage)
#         ComError = conv.ReadChannel(6, conv.data_format.voltage)
#         # print('Comerror', ComError)
#         if ComError<=THRESHOLD_V:
#             enable_mbed()
#         if RemoteE >= THRESHOLD_V:
#             print('RemoteE', RemoteE)
#             FlagEmergency=1
#             while FlagEmergency:
#                 conv.SET_DAC2(0, conv.data_format.voltage)
#                 conv.SET_DAC0(ZERO_V, conv.data_format.voltage)
#                 conv.SET_DAC1(ZERO_V, conv.data_format.voltage)
#                 ResetFSR = conv.ReadChannel(5, conv.data_format.voltage)
#                 if ResetFSR >= THRESHOLD_V:
#                     print('ResetFSR ', ResetFSR)
#                     FlagEmergency=0
#                     enable_mbed()
#                 time.sleep(0.1)

# except KeyboardInterrupt:
#     pass
# end = time.time()  # for calculate frequency

# print float(counter1) / float(end - start) # for calculate frequency


def qolo_node():
    global Comand_DAC0, Comand_DAC1
    global Xin, FsrZero, FsrK
    # Setting ROS Node
    
    # Call the calibration File
    load_calibration()

    pub = rospy.Publisher('qolo', String, queue_size=3)
    rospy.init_node('qolo_user', anonymous=True)
    rate = rospy.Rate(20) #  20 hz

    while not rospy.is_shutdown():

        control()   # Function of control for Qolo
        
        RemoteE = conv.ReadChannel(7, conv.data_format.voltage)
        
        ComError = conv.ReadChannel(6, conv.data_format.voltage)
        # print('Comerror', ComError)
        if ComError<=THRESHOLD_V:
            enable_mbed()
        if RemoteE >= THRESHOLD_V:
            print('RemoteE', RemoteE)
            FlagEmergency=1
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

        # hello_str = "hello Qolo %s" % rospy.get_time()
        # RosMassage = "%s %s %s %s %s %s %s %s %s %s" % (Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9])
        RosMassage = "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s" % (Comand_DAC0, Comand_DAC1, Out_v, Out_w, Out_CP, Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9])
        rospy.loginfo(RosMassage)

        pub.publish(RosMassage)
        rate.sleep()


if __name__ == '__main__':
    try:
        qolo_node()
    except rospy.ROSInterruptException:
        pass

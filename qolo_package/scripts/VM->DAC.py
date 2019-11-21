#! /usr/bin/env python

import HighPrecision_ADDA_Double as converter
import time
import math
from itertools import groupby
# import threading
# import RPi.GPIO as GPIO
import numpy as np
# from scipy.signal import butter, lfilter, freqz
# import matplotlib.pyplot as plt
import heapq
import logging
from logging import handlers
import mraa
import signal
from std_msgs.msg import String
import rospy

conv = converter.AD_DA()

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
DISTANCE = 0.62  # distance bettween two wheels
RADIUS = 0.304/2 # meter
W_ratio = 2

Command_V = 2500
Command_W = 2500
Comand_DAC0 = 0
Comand_DAC1 = 0

counter1 = 0
number = 100


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


def transformTo_Lowevel(data):
    # print('received ', Command_V, Command_W)
    print data
    # global DISTANCE, RADIUS
    # # Command_W = 5000
    # MaxSpeed = 1.44 # 5.44 # max Qolo speed: km/h
    # Max_motor_v = MaxSpeed*1000/3600/RADIUS/(2*np.pi)*60*GEAR # max motor speed: 1200 rpm
    # motor_v = 2*Max_motor_v*Command_V/5000 - Max_motor_v
    # motor_w = (2*Max_motor_v/(DISTANCE/2)*Command_W/5000 - Max_motor_v/(DISTANCE/2)) / W_ratio
    # # print("left wheel = ",motor_v, "right wheel = ",motor_w)
    # rpm_L = motor_v - DISTANCE*motor_w/2
    # rpm_R = motor_v + DISTANCE*motor_w/2
    # # print("left wheel = ",rpm_L, "right wheel = ",rpm_R)
    # Command_L = 5000*rpm_L/2400 + ZERO_V
    # Command_R = 5000*rpm_R/2400 + ZERO_V
    # # print('transformed ', Command_L, Command_R)
    # Command_L = round(Command_L, 2)
    # Command_R = round(Command_R, 2)
    # # print('transformed ', Command_L, Command_R)
    # return Command_L, Command_R


# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

def main():
    # pub = rospy.Publisher('V-W', String, queue_size=10)
    rospy.init_node('Low_lever_DAC', anonymous=True)

    while not rospy.is_shutdown():
        rospy.Subscriber('/V-W', String, transformTo_Lowevel)
        
        # hello_str = str(Command_V) ,str(Command_W), "%s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

        # RemoteE = conv.ReadChannel(7, conv.data_format.voltage)
        # ComError = conv.ReadChannel(6, conv.data_format.voltage)
        # # print('Comerror', ComError)
        # if ComError<=THRESHOLD_V:
        #     enable_mbed()
        # if RemoteE >= THRESHOLD_V:
        #     print('RemoteE', RemoteE)
        #     FlagEmergency=1
            
        #     while FlagEmergency:
        #         conv.SET_DAC2(0, conv.data_format.voltage)
        #         conv.SET_DAC0(ZERO_V, conv.data_format.voltage)
        #         conv.SET_DAC1(ZERO_V, conv.data_format.voltage)
        #         ResetFSR = conv.ReadChannel(5, conv.data_format.voltage)
        #         if ResetFSR >= THRESHOLD_V:
        #             print('ResetFSR ', ResetFSR)
        #             FlagEmergency=0
        #             enable_mbed()
        #         time.sleep(0.1)

        rospy.sleep(0.1)
        # rospy.spin()
        
start = time.time()  # for calculate frequency
try:
    # enable_mbed()
    main()

except KeyboardInterrupt:
    pass
end = time.time()  # for calculate frequency

print float(counter1) / float(end - start) # for calculate frequency

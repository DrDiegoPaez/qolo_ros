#! /usr/bin/env python3

#########  ROS version of Remote Joystick with safety ##########
##### Author: Diego Paez G.
##### Data: 2021/02/15

##### This script subscribes to a webserver
## for receiving a virtual joystick commands 
## from any device within the internal network of the robot

import time
import math
import os
# import threading
# import RPi.GPIO as GPIO
import numpy as np
import signal
import datetime
import rospy
import logging
from logging import handlers

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension 

import re
import os.path
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
# FLAG for fully manual control (TRUE) or shared control (FALSE)
#Tonado server port
try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    rospy.logerr("Cannot set niceness for the process...")
    rospy.logerr("Run the script as sudo...")

pub_remote = rospy.Publisher('qolo/modulation_command', Float32MultiArray, queue_size=1)
data_remote = Float32MultiArray()

### ---------- GLOBAL VARIABLES ---------- ####
PORT = 8080
Max_V = 0.7
Max_W = 0.7

level_relations = {
        # 'debug':logging.DEBUG,
        'info':logging.INFO,
        # 'warning':logging.WARNING,
        # 'error':logging.ERROR,
        # 'crit':logging.CRITICAL
    }

# Tornado Folder Paths
settings = dict(
    template_path = os.path.join(os.path.dirname(__file__), "templates"),
    static_path = os.path.join(os.path.dirname(__file__), "static")
    )


class MainHandler(tornado.web.RequestHandler):
  def get(self):
     print ("[HTTP](MainHandler) User Connected.")
     self.render("joy.html")

    
class WSHandler(tornado.websocket.WebSocketHandler):
  def open(self):
    print ('[WS] Connection was opened.')

 
  def on_message(self, message):
    print ('[WS] Incoming message:'), message
    print (type(message)) # result = re.findall(r"[-+]?\d*\.\d+|\d+", message)
    result = re.findall(r"[-\d]+", message) 
    # print (result)
    Output = [((float(i)/100)) for i in result]
    Ctime = round(time.clock(),4)
    publishJoystick(Ctime,Output[1]*Max_V, Output[0]*Max_W)
    print ('Joystick =', Output[1]*Max_V, Output[0]*Max_W)


  def on_close(self):
    # publishJoystick(0., 0., 0.)
    # print ('Joystick = [0 0]')
    print ('[WS] Connection was closed.')

application = tornado.web.Application([
  (r'/', MainHandler),
  (r'/ws', WSHandler),
  # (r"/(.*)", tornado.web.StaticFileHandler,
  #            {"path": r"{0}".format(os.path.join(os.path.dirname(__file__), "static"))}),
  ], **settings)

def publishJoystick(time,Vel,Omega):
    global pub_remote, data_remote
    data_remote.data = [time,Vel,Omega]
    pub_remote.publish(data_remote)
    rospy.loginfo(data_remote)


def joystick_control():
    global pub_remote, data_remote
    try:
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(PORT)
        main_loop = tornado.ioloop.IOLoop.instance()
        data_remote.layout.dim.append(MultiArrayDimension())
        data_remote.layout.dim[0].label = 'Joystick Commands [V, W]'
        data_remote.layout.dim[0].size = 3
        data_remote.data = [0]*3

        rospy.init_node('qolo_joystick', anonymous=True)
        rate = rospy.Rate(50) #  50 hz
        print ("Tornado Server started")
        main_loop.start()

    except:
        publishJoystick(0., 0., 0.)
        print ('Joystick = [0 0]')
        time.sleep(0.1)
        publishJoystick(0., 0., 0.)
        print ('Joystick = [0 0]')
        print ("Exception triggered - Tornado Server stopped.")
        # GPIO.cleanup()

# for interruptions
def exit(signum, frame):
    # Stop_Thread_Flag = True
    # cleanup_stop_thread()
    print('---> You chose to interrupt')
    quit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    try:
        joystick_control()
    except rospy.ROSInterruptException:
        time.sleep(0.1)
        pass
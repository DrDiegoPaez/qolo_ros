#!/usr/bin/env python
import rospy
from crowdbotsim.msg import CrowdStamped
import math
import signal
import sys

reached_goal_radius = 1.0
reached_goal_time = None
initial_timestamp = None

def print_result():
    mean = 0.0
    N = 0
    for rgt in reached_goal_time:
        if rgt != None:
            mean += rgt
            N += 1
    mean /= N
    std = 0.0
    for rgt in reached_goal_time:
        if rgt != None:
            std += (rgt-mean)*(rgt-mean)
    std = math.sqrt(1.0/(N - 1.0)*std)
    print ("Total number of agents: ", len(reached_goal_time))
    print ("Number of agents that reached their goal: ", N)
    print ("For those, average reaching time: ", mean)
    print ("With standard deviation: ", std)

def signal_handler(sig, frame):
    print_result()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def callback(msg):
    global reached_goal_time
    global initial_timestamp
    if reached_goal_time == None:
        reached_goal_time = [None]*len(msg.crowd)
    if initial_timestamp == None:
        initial_timestamp = msg.header.stamp
    for i in range(msg.crowd):
        ped = msg.crowd[i]
        diff_x = ped.pose.pose.position.x - ped.goal.x
        diff_y = ped.pose.pose.position.y - ped.goal.y
        distance = math.sqrt(diff_y*diff_y + diff_x*diff_x)
        if distance < reached_goal_radius and reached_goal_time[i] == None:
            reached_goal_time[i] = msg.header.stamp - initial_timestamp

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("crowd", CrowdStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
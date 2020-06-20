#!/usr/bin/env python
import rospy
from crowdbotsim.msg import CrowdStamped
import math
import signal
import sys

reached_goal_radius = 1.0
reached_goal_time = None
initial_timestamp = None
mean_velocities = None
duration = 0.0

def print_result_reaching_time():
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
    print ("-----Result for reaching time-------")
    print ("Total number of agents: ", len(reached_goal_time))
    print ("Number of agents that reached their goal: ", N)
    print ("For those, average reaching time: ", mean)
    print ("With standard deviation: ", std)

def print_result_mean_velocity():
    mean = 0.0
    for mv in mean_velocities:
        mean += mv
    mean /= len(mean_velocities)
    std = 0.0
    for mv in mean_velocities:
        std += (mv-mean)*(mv-mean)
    std = math.sqrt(1.0/(len(mean_velocities) - 1.0)*std)
    print ("-----Result for mean velocities-------")
    print ("Total number of agents: ", len(mean_velocities))
    print ("For those, average velocity: ", mean)
    print ("With standard deviation: ", std)  

def signal_handler(sig, frame):
    print_result_reaching_time()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def callback(msg):
    global reached_goal_time
    global initial_timestamp
    global duration
    global mean_velocities
    if reached_goal_time == None:
        reached_goal_time = [None]*len(msg.crowd)
    if initial_timestamp == None:
        initial_timestamp = msg.header.stamp
    for i in range(msg.crowd):
        ped = msg.crowd[i]
        diff_x = ped.position.x - ped.goal.x
        diff_y = ped.position.y - ped.goal.y
        distance = math.sqrt(diff_y*diff_y + diff_x*diff_x)
        if distance < reached_goal_radius and reached_goal_time[i] == None:
            reached_goal_time[i] = msg.header.stamp - initial_timestamp

    if mean_velocities == None:
        mean_velocities = [0.0]*len(msg.crowd)

    dt = msg.header.stamp - initial_timestamp - duration
    for i in range(msg.crowd):
        ped = msg.crowd[i]
        w_old = duration/(dt + duration)
        w_new = dt/(dt + duration)
        v_x2 = ped.velocity.linear.x*ped.velocity.linear.x
        v_y2 = ped.velocity.linear.y*ped.velocity.linear.y
        v_z2 = ped.velocity.linear.z*ped.velocity.linear.z
        v = math.sqrt(v_x2 + v_y2 + v_z2)
        mean_velocities[i] = w_old*mean_velocities[i] + w_new*v

    duration = msg.header.stamp - initial_timestamp


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("crowd", CrowdStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
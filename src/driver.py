#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.
# @author Jacqueline Zhou @email jianingzhou@brandeis.edu
import rospy
import random
import numpy as np
#from state_definitionsn import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from math import pi
#------------------------------------------------------------------------------------------
#==============================## functions and callbacks ##==============================
#------------------------------------------------------------------------------------------
#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg
    print(t_pid.linear.x, t_pid.angular.z)

# randomly generate 1 or -1
def rand_one():
    return 1 if random.random() < 0.5 else -1
#------------------------------------------------------------------------------------------
#===========================## init node and define variables ##===========================
#------------------------------------------------------------------------------------------
#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
r = 10
rate = rospy.Rate(r)

# Linear speed of the robot
LINEAR_SPEED = 0.3
# Angular speed of the robot
ANGULAR_SPEED = pi/6

# following left wall
LEFT = 1
# following right wall
RIGHT = 2
# turning
TURN = 3
# following state
WANDER = 4

# starting state
state = 0

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

#------------------------------------------------------------------------------------------
#=====================================## while loop ##=====================================
#------------------------------------------------------------------------------------------
# buffer loop
while t_pid == Twist() or state == 0: continue

print("STARTING")

while not rospy.is_shutdown():
    print("STATE: ", state)
    if state == LEFT or state == RIGHT or state == TURN:
        t_pub = t_pid
    elif (state == WANDER):
        # have the robot wander around
        t_pub.linear.x = LINEAR_SPEED
        t_pub.angular.z = random.randrange(4) * rand_one() * ANGULAR_SPEED
    else:
        print("STATE NOT FOUND")
    pub_vel.publish(t_pub)
    rate.sleep()

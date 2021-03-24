#!/usr/bin/env python

# This is a PID controller to determine twist values
# @author Jacqueline Zhou @email jianingzhou@brandeis.edu

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from math import pi, sqrt
from prrexamples.msg import Filtered_Laserscan
#------------------------------------------------------------------------------------------
#==============================## functions and callbacks ##==============================
#------------------------------------------------------------------------------------------
#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

# scan_values_handler subscriber callback
def sub_svh_cb(msg):
    global filtered_laserscan
    filtered_laserscan = msg

# calculate errors and pid components
def calc_pid():
    global curr_error
    global prev_error
    global sum_error
    global pid
    error = min(filtered_laserscan.filtered_ranges) - (min(filtered_laserscan.filtered_ranges[60:120])+min(filtered_laserscan.filtered_ranges[240:300]))/2
    prev_error = curr_error
    curr_error = error
    sum_error += curr_error * dT
    pid = get_pid(curr_error, prev_error, sum_error)
    return pid

# calculate pid value
def get_pid(curr, prev, sum):
    p_component = curr
    d_component = (curr - prev)/dT
    i_component = sum
    return P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component
#------------------------------------------------------------------------------------------
#===========================## init node and define variables ##===========================
#------------------------------------------------------------------------------------------
# Init node
rospy.init_node('pid')

# Create publisher for suggested twist objects
pub = rospy.Publisher('pid_twist', Twist, queue_size = 1)

# subscribe to scan_values_handler 
sub_svh = rospy.Subscriber('cross_err', Filtered_Laserscan, sub_svh_cb)
# subscribe to state
sub_state = rospy.Subscriber('state', Int16, cb_state)

# initialize Filtered_Laserscan object
filtered_laserscan = Filtered_Laserscan()

# Twist and rate object
t = Twist()
r = 10
rate = rospy.Rate(r)

# allowed error
err = 0.5

# initialize variables for pid calculation
EXP_DIST = 1.0
dT = 1.0 / r
curr_error = 0.0
prev_error = 0.0
sum_error = 0.0
pid = 0.0

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

# Linear speed of the robot
LINEAR_SPEED = 0.3
# Angular speed of the robot
ANGULAR_SPEED = pi/6

# Multipliers used to tune the PID controller
# Proportional constant
P_CONSTANT = 0.8
# Integral constant
I_CONSTANT = 0.01
# Derivative constant
D_CONSTANT = 0.2
#------------------------------------------------------------------------------------------
#=====================================## while loop ##=====================================
#------------------------------------------------------------------------------------------
# buffer loop
while filtered_laserscan == Filtered_Laserscan(): continue

while not rospy.is_shutdown():
    if state == LEFT:
        # use pid
        pid = calc_pid()
        t.angular.z = ANGULAR_SPEED * pid
        t.linear.x = LINEAR_SPEED
    elif state == RIGHT:
        # use pid
        pid = calc_pid()
        t.angular.z = ANGULAR_SPEED * pid
        t.linear.x = LINEAR_SPEED
    elif state == TURN:
        if min(filtered_laserscan.filtered_ranges[0:100]) < min(filtered_laserscan.filtered_ranges[260:360]):
            # turn right
            t.angular.z = ANGULAR_SPEED * (-1)
            t.linear.x = LINEAR_SPEED*0.6
        else:
            # turn left
            t.angular.z = ANGULAR_SPEED
            t.linear.x = LINEAR_SPEED*0.6
    elif state == WANDER:
        # whatever, handled by driver.py
        t.angular.z = 0
        t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 
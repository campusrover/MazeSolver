#!/usr/bin/env python

# This processes all of the scan values
# @author Jacqueline Zhou @email jianingzhou@brandeis.edu

import rospy
import numpy as np
#from state_definitionsn import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from prrexamples.msg import Filtered_Laserscan
#------------------------------------------------------------------------------------------
#==============================## functions and callbacks ##==============================
#------------------------------------------------------------------------------------------
# Process all the data from the LIDAR
def cb(msg):
    global state
    # calculate msg to publish to pid
    msg_pid = calc_laserscan(msg)
    # publish msg_pid
    pub_svh.publish(msg_pid)
    # define the state by processed ranges info
    state = state_define(region_check)
    # publish state
    pub_state.publish(state)

# calculate and compile Filtered_Laserscan object
def calc_laserscan(msg):
    ranges = np.array(msg.ranges)
    for i in range(360):
        if ranges[i] == float('inf') or ranges[i] < msg.range_min:
            ranges[i] = 4
    calc_regions(ranges)
    msg_new = Filtered_Laserscan()
    msg_new.filtered_ranges = ranges
    return msg_new

# define regions
def calc_regions(ranges):
    global regions
    # calculate different region min
    North = np.append(ranges[0:30], ranges[330:360])
    regions = {
        "N":     min(North),                                # north
        "NW":    min(ranges[30:60]),                        # north-west
        "W":     min(ranges[60:120]),                       # west
        "SW":    min(ranges[120:150]),                      # south-west
        "S":     min(ranges[150:210]),                      # south
        "SE":    min(ranges[210:240]),                      # south-east
        "E":     min(ranges[240:300]),                      # east
        "NE":    min(ranges[300:330]),                      # north-east
    }
    global region_check
    # make things easier by creating a bool dict
    region_check = {
        "N":     min(North) < detect_dist - 2.4,            # north
        "NW":    min(ranges[30:60]) < detect_dist,          # north-west
        "W":     min(ranges[60:120]) < detect_dist,         # west
        "SW":    min(ranges[120:150]) < detect_dist,        # south-west
        "S":     min(ranges[150:210]) < detect_dist,        # south
        "SE":    min(ranges[210:240]) < detect_dist,        # south-east
        "E":     min(ranges[240:300]) < detect_dist,        # east
        "NE":    min(ranges[300:330]) < detect_dist,        # north-east
    }

# define state by ranges
def state_define(region_check):
    # wall on the left
    #if region_check["W"] and regions["W"] >= regions["E"] and not region_check["N"]:
    if region_check["W"] and not region_check["N"]:
        return LEFT
    #elif region_check["E"] and regions["E"] >= regions["W"] and not region_check["N"]:
    elif region_check["E"] and not region_check["N"]:
        return RIGHT
    elif region_check["N"]:
        return TURN
    else:
        return WANDER
#------------------------------------------------------------------------------------------
#===========================## init node and define variables ##===========================
#------------------------------------------------------------------------------------------
# Init node
rospy.init_node('scan_values_handler')

# Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

# Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
pub_svh = rospy.Publisher('cross_err', Filtered_Laserscan, queue_size = 1)

# Rate object
rate = rospy.Rate(10)

ranges = []

# following left wall
LEFT = 1
# following right wall
RIGHT = 2
# turning
TURN = 3
# following state
WANDER = 4

# initialize error objects
err = 0.5
detect_dist = 3.5
cross_error = 0

# starting state
state = 0

# regions
regions = {}
region_check = {}
#------------------------------------------------------------------------------------------
#=====================================## while loop ##=====================================
#------------------------------------------------------------------------------------------
# buffer loop
while state == 0 or regions == {} or region_check == {}: continue

# Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 
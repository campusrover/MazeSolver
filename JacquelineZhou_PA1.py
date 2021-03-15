#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi


def outnback():
    # set name of node
    rospy.init_node('OutnBack', anonymous=False)
    # initialize Publisher
    sim_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # set rate at 50Hz
    r = rospy.Rate(50)
    # initialize message
    twist = Twist()

    # goal is to travel 1.5m
    distance = 1.5
    # speed of moving straight
    speed = 0.3
    # goal is to rotate pi
    angle = pi
    # speed of rotating
    rotate_speed = 0.5

    # call move_forward() to move in straight line
    move_forward(sim_pub, distance, speed, twist, r)

    # call rotate() to rotate
    rotate(sim_pub, angle, rotate_speed, twist, r)

    # call move _forward() to move in straight line
    move_forward(sim_pub, distance, speed, twist, r)


def move_forward(sim_pub, distance, speed, twist, r):
    # calculate rate of publishing command
    tsec = int(distance / speed * 50)
    # initialize command
    twist = set_forward(speed)
    # use a for loop for publishing command
    for t in range(tsec):
        # publish command
        sim_pub.publish(twist)
        # prepare for next command
        r.sleep()
    # reset empty command
    twist = Twist()
    # publish stop command
    sim_pub.publish(Twist())
    # stop the robot
    rospy.sleep(1)


def rotate(sim_pub, angle, rotate_speed, twist, r):
    # calculate rate of publishing command
    tsec = int(angle / rotate_speed * 50)
    # initialize command
    twist = set_rotate(rotate_speed)
    # use a for loop for publishing command
    for t in range(tsec):
        # publish command
        sim_pub.publish(twist)
        # prepare for next command
        r.sleep()
    # reset empty command
    twist = Twist()
    # publish stop command
    sim_pub.publish(Twist())
    # stop the robot
    rospy.sleep(1)


# return a message with a forward speed
def set_forward(speed):
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    return twist


# return a message with an angular speed
def set_rotate(degree):
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = degree
    return twist


if __name__ == '__main__':
    try:
        outnback()
    except rospy.ROSInterruptException:
        pass
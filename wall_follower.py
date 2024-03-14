#!/usr/bin/env python
# --- wall_follower.py       Version 1.3     Mohammad Qaddoura MFET 442
#   --- 
#   ---
#   --- Copyright (c) 2008, Willow Garage, Inc.
#   ---
#   --- 05/21/20 RMV    Initial coding. Copied from Writing a Simple Publisher and Subscriber for rospy_tutorials. v1.0
#   --- 03/05/24 	    Created a subscriber for the scan topic
#   --- 03/06/24        Created the PD controller for the linear y velocity
#   --- 03/11/24        Created the PD controller for angular z velocity

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## A program that reads the sensor values from the LIDAR and publishes commands
## to a robot to follow a wall from 1 meter away

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

distance = 0
aDistance = 1
# Setting initial distances for linear and angular velocity formulas

def callback(msg):
    # callback(msg)
    #
    # The callback function takes input msg from the different
    # sensor topics included in the omniveyor and stores the desired
    # value in that sensors range in the distance variable.
    # In this case the topic is /scan.

    global distance
    global aDistance
    distance = msg.ranges[719]
    aDistance = msg.ranges[540]

def listener():
    global distance
    global aDistance
    # def listener():
    # The listener function creates an anonymous node to
    # store data from the /scan topic.
    # It then creates a subscriber which subscribes to the
    # topic /scan which has the LaserScan data structure, and
    # performs the callback function.

    rospy.init_node('revised_scan', anonymous=True)
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('/scan', LaserScan, callback)
    # Creating the subscriber which subscirbes to the /scan topic
    # which calls the callback function on the LaserScan datastructure it receives

    pubTopic = '/cmd_vel'
    # Storing the topic name in a variable called
    # pubTopic so it can easily be accessed

    pub = rospy.Publisher(pubTopic, Twist)
    # Creating a publisher which publishes commands via the Twist() function
    # to the /cmd_vel topic that is used to control the velocities of the robot

    #!!!!!!!!!!rospy.loginfo("I will publish to the topic %s", pubTopic)
    # Printing the topic that 


    while not rospy.is_shutdown():
        # Creating a while loop so that the robot can opperate continuously
        # the condition is until the user stops rospy from running

        adjustment = Twist()
        # A variable named adjustment is assigned to be part of the Twist class
        # which controls the angular and linear velocities of the robot

        readRange = distance
        # setting a variable to store the value of distance from the wall of the last laser

        readAngle = aDistance
        # setting a variable to store the value of the distance from the wall of the laser 45
        # degrees apart from the last laser

        kpLinear = .5 # Setting the magnitude of correction of the linear correction function,
                      # this part of the function is responsible for approaching the setpoint
                      # the higher this factor is the faster the robot will approach the setpoint

        kdLinear = 100 # Setting the kd factor controls the rate of change of the linear correction function,
                       # This increases stability of the correction and prevents overshooting
                       # as its influence of the system is proportional to the change
                       # in the robots position due to the correction.
                       # If this factor is too high or too low the system will over-oscillate around the setpoint.

        kdAngular = 10 # Setting the factor that controls the angular velocity of the robot

        adjustment.linear.x = 1.0
        # Setting the x velocity to be a constant value

        if readRange != "inf":
            # A line to make sure the value received from the LIDAR is
            # a floating number, in order to avoid errors

            dInitial = float(readRange)    # Assigning the initial value to the
                                    # immediate reading the LIDAR records

            rospy.sleep(.01)        # delaying for a 100th of a second, in other words setting frequency to 100 Hz

            dFinal = float(readRange)      # Assigning the final value to the
                                    # value the LIDAR reads after a 100th of a second,
                                    # setting our PD interval to a 100th of a second

        if readAngle != "inf":
            # This if statement works the same as the previous one
            # however, it is used to track the distance that is
            # 45 degrees from the last laser

            aInitial = float(readAngle)

            rospy.sleep(.01)

            aFinal = float(readAngle)

        adjustment.linear.y = -kpLinear*(1.0 - dFinal) - kdLinear*(0.0 - (dFinal - dInitial)/.01)
        # Setting the y velocity to adjust based on the LIDAR's distance value

        adjustment.angular.z = -kdAngular*(0.0 - (((dFinal * math.cos(math.atan((aFinal * math.cos(.785398) - dFinal) / readAngle * math.sin(.785398)))) - (dInitial * math.cos(math.atan((aInitial * math.cos(.785398) - dInitial) / readAngle * math.sin(.785398)))))/.01))      
        # Setting the z angular velocity, this error formula compares the distance from the robot to the wall currently, to what it should be when it is perfectly parallel to the wall

        rospy.loginfo(adjustment)
        # Used to track the velocity values

        pub.publish(adjustment)
        # Publishes the command to the robot in order to adjust its velocity
        # and maintain the desired distance and alignment with the wall


if __name__ == '__main__':
    listener()

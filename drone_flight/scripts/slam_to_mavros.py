#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 03 05:39:30 2023

@author: mason
"""

''' import libraries '''
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class slam_to_mavros_pub():
    def __init__(self):
        rospy.init_node('slam_to_mavros', anonymous=True)
        self.my_pose = rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=2)

        self.rate = rospy.Rate(1)

    def odom_cb(self, msg):
        data = PoseStamped()
        data.header = msg.header
        data.pose = msg.pose.pose
        # data.pose.position.x = data.pose.position.x + offset_x # if needed, offset
        # data.pose.position.y = data.pose.position.y + offset_y # if needed, offset
        # data.pose.position.z = data.pose.position.z + offset_z # if needed, offset
        self.pose_pub.publish(data)

''' main '''
pub_class = slam_to_mavros_pub()

if __name__ == '__main__':
    while 1:
        try:
            pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
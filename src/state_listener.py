#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

#Simple listener to get ground truth information from simulation

def odometry_callback(msg):
    print(msg.pose.pose)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('ground_truth/state', Odometry, odometry_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

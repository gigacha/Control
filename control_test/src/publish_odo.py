#!/usr/bin/env python
# license removed for brevity

import rospy
from nav_msgs.msg import Odometry
from time import sleep

pub = rospy.Publisher('/pub_odo', Odometry, queue_size = 1)
rospy.init_node('pub_node')
rospy.loginfo("-------a start!-------")

# while True:
    
#     cur_x = input()
#     cur_y = input()
#     cur_yaw = input()
    
#     msg_odo = Odometry()
#     msg_odo.pose.pose.position.x = cur_x
#     msg_odo.pose.pose.position.y = cur_y
#     msg_odo.pose.pose.orientation.x = cur_yaw

#     rospy.loginfo("==============")
#     rospy.loginfo(msg_odo.pose.pose.position.x)
#     rospy.loginfo(msg_odo.pose.pose.position.y)
#     rospy.loginfo(msg_odo.pose.pose.orientation.x)
#     pub.publish(msg_odo)
# 


while True:
    i=200
    cur_x = input()
    cur_y = input()
    cur_yaw = input()
    while i > 0:
        sleep(0.1)
        i-=1
        msg_odo = Odometry()
        msg_odo.pose.pose.position.x = 0
        msg_odo.pose.pose.position.y = 0
        msg_odo.pose.pose.orientation.x = 0
        msg_odo.pose.pose.orientation.y = 0

        rospy.loginfo("==============")
        rospy.loginfo(msg_odo.pose.pose.position.x)
        rospy.loginfo(msg_odo.pose.pose.position.y)
        rospy.loginfo(msg_odo.pose.pose.orientation.x)
        pub.publish(msg_odo)

    i=100
    while i > 0:
        sleep(0.1)
        i-=1
        msg_odo = Odometry()
        msg_odo.pose.pose.position.x = 5
        msg_odo.pose.pose.position.y = 5
        msg_odo.pose.pose.orientation.x = 0
        msg_odo.pose.pose.orientation.y = 0
        
        rospy.loginfo("==============")
        rospy.loginfo(msg_odo.pose.pose.position.x)
        rospy.loginfo(msg_odo.pose.pose.position.y)
        rospy.loginfo(msg_odo.pose.pose.orientation.x)
        pub.publish(msg_odo)

    i=100
    while i > 0:
        sleep(0.1)
        i-=1
        msg_odo = Odometry()
        msg_odo.pose.pose.position.x = -5
        msg_odo.pose.pose.position.y = 5
        msg_odo.pose.pose.orientation.x = 0
        msg_odo.pose.pose.orientation.y = 0

        rospy.loginfo("==============")
        rospy.loginfo(msg_odo.pose.pose.position.x)
        rospy.loginfo(msg_odo.pose.pose.position.y)
        rospy.loginfo(msg_odo.pose.pose.orientation.x)
        pub.publish(msg_odo)


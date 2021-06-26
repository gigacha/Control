#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def msgCallback(msg):
    rospy.loginfo(msg.pose.pose.position.x)
    rospy.loginfo(msg.pose.pose.position.y)
    rospy.loginfo(msg.pose.pose.orientation.x)
    rospy.loginfo(msg.pose.pose.orientation.y)
    rospy.loginfo(msg.pose.pose.orientation.z)
    rospy.loginfo(msg.pose.pose.orientation.w)
    rospy.loginfo(msg.twist.twist.linear.x)
    rospy.loginfo(msg.twist.twist.linear.y)
    rospy.loginfo(msg.twist.twist.angular.x)
    rospy.loginfo(msg.twist.twist.angular.y)

rospy.init_node('sub_node')
rospy.loginfo("-------c start!-------")
rospy.Subscriber("/pub_odo", Odometry, msgCallback)
rospy.spin()

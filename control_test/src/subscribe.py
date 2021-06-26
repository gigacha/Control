#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def msgCallback(msg):
    point_x = msg.position.x
    point_y = msg.position.y
    rospy.loginfo("=================")
    rospy.loginfo(point_x)
    rospy.loginfo(point_y)
    rospy.loginfo(msg.orientation.w)
    rospy.loginfo(msg.orientation.x)
    rospy.loginfo(msg.orientation.y)
    rospy.loginfo(msg.orientation.z)


rospy.init_node('sub_node')
rospy.loginfo("-------c start!-------")
rospy.Subscriber("/pub_pos", Pose, msgCallback)
rospy.loginfo("-------before spin-------")
#rospy.spin()
rospy.wait_for_message("pub_pos", Pose)
rospy.loginfo("-------after spin-------")

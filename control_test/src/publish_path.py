#!/usr/bin/env python
# license removed for brevity

import rospy

from geometry_msgs.msg import PoseArray
import geometry_msgs.msg

pub = rospy.Publisher('/pub_path', PoseArray, queue_size = 1)
rospy.init_node('pub_node')
rospy.loginfo("-------a start!-------")

rate = rospy.Rate(1) #10hz 1/10 = 0.1s
n = 0
#while not rospy.is_shutdown():
#    if(n==11):
#        n=0
p = PoseArray()
size_arr = 20
for ii in range(size_arr):
    pose = geometry_msgs.msg.Pose()
    point_P = (ii, ii, 0)
    pose.position = geometry_msgs.msg.Point(*point_P)
    p.poses.append(pose)
rospy.loginfo(p.poses[0].position.x)
rospy.loginfo(p.poses[0].position.y)
rospy.loginfo(p.poses[2].position.x)
rospy.loginfo(p.poses[2].position.y)

pub.publish(p)

n += 1
rate.sleep() #0.1s

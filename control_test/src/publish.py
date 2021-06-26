#!/usr/bin/env python
# license removed for brevity

import rospy

from geometry_msgs.msg import Pose

x = [1,2,3,4,5, 6, 7, 8, 9, 10, 10, 11]
y = [1,1,2,2,3, 3, 4, 4, 7, 8, 10, 11]

pub = rospy.Publisher('/pub_pos', Pose, queue_size = 1)
rospy.init_node('pub_node')
rospy.loginfo("-------a start!-------")

rate = rospy.Rate(1) #10hz 1/10 = 0.1s
n = 0
while not rospy.is_shutdown():
    if(n==11):
        n=0
    p = Pose()
    p.position.x = x[n]
    p.position.y = y[n]
    p.orientation.w = n
    p.orientation.x = x[n]
    p.orientation.y = y[n]
    p.orientation.z = x[n]
    rospy.loginfo("==============")
    rospy.loginfo(p.position.x)
    rospy.loginfo(p.position.y)
    rospy.loginfo(p.orientation.w)
    rospy.loginfo(p.orientation.x)
    rospy.loginfo(p.orientation.y)
    rospy.loginfo(p.orientation.z)

    pub.publish(p)
    n += 1
    rate.sleep() #0.1s

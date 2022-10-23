#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    robot_pose_pub = rospy.Publisher('/robot_pose', Float64MultiArray, queue_size=10)
    robot_orientation_pub = rospy.Publisher('/robot_orientation', Float64MultiArray, queue_size=10)

    robot_pose = Float64MultiArray()
    robot_orientation = Float64MultiArray()
    rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            print(trans)
            print(rot)

            robot_pose.data = trans
            robot_orientation.data = rot
            robot_pose_pub.publish(robot_pose)
            robot_orientation_pub.publish(robot_orientation)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
        rate.sleep()


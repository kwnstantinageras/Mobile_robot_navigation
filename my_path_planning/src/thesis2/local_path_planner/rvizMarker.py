#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from nav_msgs.msg import Path



class MarkerBasics(object):

    def __init__(self):
        self.init_marker(index=0, z_val=0)
        rospy.Subscriber("/path_points", Point, self.path_points_visualization)

        self.points = []
        self.marker_object_publisher = rospy.Publisher('/marker_basic', Marker, queue_size=10)
        self.rate = rospy.Rate(1)





    def path_points_visualization(self, point):
        self.points.append(point)



    def init_marker(self, index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "map"
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = "haro"
        self.marker_object.id = index
        self.marker_object.type = Marker.POINTS
        self.marker_object.action = Marker.ADD

        count = 0
        self.marker_object.points = []




        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 0.1
        # self.marker_object.scale.y = 1.0
        # self.marker_object.scale.z = 1.0

        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it forever, 0, otherwise seconds before disappearing
        self.marker_object.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            for i in range(len(self.points)):
                print(self.points[i])
                my_point = Point()
                my_point.x = self.points[i].x
                my_point.y = self.points[i].y
                my_point.z = 0
                self.marker_object.points.append(my_point)
            self.marker_object_publisher.publish(self.marker_object)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass
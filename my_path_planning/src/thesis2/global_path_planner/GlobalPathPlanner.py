#!/usr/bin/env python

from global_path_planner.Dijkstra import dijkstra
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from global_path_planner.NodeDijkstra import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point


class GlobalPathPlanner:
    # start position from the odometry/amcl and goal depends on gesture
    def __init__(self, goal, nodes):
        # start is a Node
        self.start = None  # start position
        self.goal = goal  # goal position
        self.nodes = nodes

        #self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #self.odom_subscriber = rospy.Subscriber('/RosAria/pose', Odometry, self.odom_callback)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        #FOR PATH VISUALIZATION!!!!!!!!!!!
        self.point = Point()
        self.path_points_pub = rospy.Publisher('/path_points', Point, queue_size=10)

    """
    def odom_callback(self, odom):
        # self.start = odom.pose.pose

        # KOVW TA DEKADIKA TWN SHMEIWN
        self.start = Node(odom.pose.pose.position.x, odom.pose.pose.position.y)

        # self.start = odom.pose.pose.position
        #print("start point is:")
        #print(self.start.x)
        #print(self.start.y)
    """


    def pose_callback(self, amcl_pose):

        self.start = Node(amcl_pose.pose.pose.position.x, amcl_pose.pose.pose.position.y)


    # returns a list with the path points if it exists, otherwise there is no path to goal
    def plan(self):

        rospy.sleep(1)

        for node in self.nodes:
            print("%s , %s, %s" % (node.x, node.y, node.weight))
        path = dijkstra(self.nodes, self.start, self.goal)

        # FOR THE MARKER VISUALIZATION IN RVIZ
        for node in path:
            self.point.x = node.x
            self.point.y = node.y
            self.path_points_pub.publish(self.point)
            rospy.sleep(0.01)



        return path



#!/usr/bin/env python
import math
import time

flag = 0


from global_path_planner.GlobalPathPlanner import GlobalPathPlanner
from local_path_planner.LocalPathPlanner2 import LocalPathPlanner
from global_path_planner.GridController import GridController
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class RobotNavigation:
    def __init__(self):

        self.gesture = None
        self.goal = None

        print("Waiting for gesture..")
        rospy.init_node('RobotNavigation', anonymous=True)
        rospy.Subscriber("/gesture_class", String, self.gesture_callback)
        #self.velocity_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        #self.vel_msg = Twist()

        self.gesture_terminate = rospy.Publisher('/gesture_terminate', Twist, queue_size=10)


    def gesture_callback(self, gesture_class):
        self.gesture = gesture_class.data
        global flag

        if self.gesture == 'OK':
            flag = 1
            print("Navigation for point (3,3) has started.")

        if self.gesture == 'six':
            print("Navigation terminated.")
            flag = 0


    def start_planning(self):
        global flag
        if flag == 1:
            self.goal = (4, 4)
            # create graph from grid and pass nodes[] to globalpathplanning
            self.grid_controller = GridController()

            self.grid_controller.unoccupied_cells_from_grid()
            self.obstacle_free_grid_cells = self.grid_controller.unoccupied_cells_world_coordinates
            global_path = GlobalPathPlanner(self.goal, self.obstacle_free_grid_cells)
            path = global_path.plan()

            if not path:
                print("Path not found.")
            else:
                print("Global path planner is complete.")
                print("Robot starts going with Local path planner.")
                local_path = LocalPathPlanner(path)
                local_path.go()
            flag = 0


if __name__ == "__main__":
    try:
        navigation = RobotNavigation()
        while True:
            navigation.start_planning()
            # rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program terminated.")
    rospy.spin()

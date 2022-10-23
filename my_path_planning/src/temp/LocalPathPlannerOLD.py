#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose, Quaternion
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
import time
import tf

LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.4
TOLERANCE = 0.8
ROBOT_RADIUS = 0.22
OBSTACLE_THRESHOLD = 0.78
EXIT_STATUS_ERROR = 1
EXIT_STATUS_OK = 0

# Globals
x_robot = 0
y_robot = 0
yaw = 0
min_value_right = 0
min_value_front = 0
min_value_left = 0
is_stop_moving = False
vel_msg = Twist()


class LocalPathPlanner:
    def __init__(self, points):
        self.points = points

        # Set ROS nodes.
        # rospy.init_node('traveler', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/base_scan", LaserScan, self.clbk_laser)
        rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.pos = Pose()
        self.theta = 0
        self.start = (0, 0)
        self.goal = None
        self.x_goal = None
        self.y_goal = None
        self.sonar_data = []
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.obstacle_found = False
        self.obstacle_circumventing = None

        self.mline = None
        self.curr_line = None

        self.regions = None

    def pose_callback(self, pose_data):
        global x_robot, y_robot, yaw

        x_robot = pose_data.pose.pose.position.x
        y_robot = pose_data.pose.pose.position.y

        quaternion = (
            pose_data.pose.pose.orientation.x,
            pose_data.pose.pose.orientation.y,
            pose_data.pose.pose.orientation.z,
            pose_data.pose.pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        yaw = rpy[2]

        """
        if self.goal != None:
            self.curr_line = self.slope((x_robot, y_robot), self.goal)
        """

    def clbk_laser(self, msg):
        # 720 / 5 = 144
        regions = {
            'right': min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front': min(min(msg.ranges[288:431]), 10),
            'fleft': min(min(msg.ranges[432:575]), 10),
            'left': min(min(msg.ranges[576:713]), 10),
        }
        self.regions = regions

        # OBSTACLE DETECTION
        if self.regions['right'] < 0.2 or self.regions['fright'] < 0.2 or self.regions['front'] < 0.2 or self.regions[
            'fleft'] < 0.2 or self.regions['left'] < 0.2:
            self.obstacle_found = True
        else:
            self.obstacle_found = False
        print(self.obstacle_found)
        # self.take_action(regions)

    """
    def go(self):
        """"""
        Method to go to goal specified irrespective of the current position and orientation
        of the robot.
        """"""
        time.sleep(2.0)

        # ta points einai se world coordinates
        # phgainei apo shmeio se shmeio

        for goal in self.points:
            # tin twrini thesi tin pairnw apo to callback

            self.goal = goal

            self.x_goal = goal.x
            self.y_goal = goal.y



            # SE SHESI ME TO 2- PAEI PIO GRHGORA TO 1
            while self.is_goal_reached() == False:
                #self.rotate_to_goal()

                if self.is_towards_goal() == False:
                    self.rotate_to_goal()


                """"""
                #TO B EINAI KALUTERO APO TO A
                while self.is_towards_goal() == False:
                    self.rotate_to_goal()
                """"""
                self.move_forward()

            """"""
            while self.is_goal_reached() == False:
                self.rotate_to_goal()
                self.move_forward()
            """"""


            """"""
            while self.is_goal_reached() == False:
                self.take_actions()

            print("Goal is reached.")
            """"""
            print("Goal is reached.")
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
    """
    """
    def slope(self, p1, p2):
        """"""
        Method to calculate the slope of a line segment formed by two given points
        @arg 	p1 	first point of line segment
        @arg 	p2 	second point of line segment
        @returns 	slope of line segment
        """"""
        delta_y = p2.y - p1[1]
        delta_x = p2.x - p1[0]

        # ta goals einai tupou node

        return delta_y / delta_x if delta_x != 0 else float('inf')
    """

    def stop(self):
        """
        Method to bring robot to a halt by publishing linear and angular velocities of zero.
        """
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    """
    def new_obstacle(self):

        obstacle_position
        if new == False:
            self.new_obstacle = False
        else:
            self.new_obstacle = True
    """

    def go(self):
        """
        Method to go to goal specified irrespective of the current position and orientation
        of the robot.
        """
        time.sleep(2.0)

        # ta points einai se world coordinates
        # phgainei apo shmeio se shmeio

        for goal in self.points:
            # tin twrini thesi tin pairnw apo to callback

            self.goal = goal

            self.x_goal = goal.x
            self.y_goal = goal.y

            self.mline = self.curr_line

            """
            while not self.is_goal_reached():# and not self.new_obstacle():


                if self.is_towards_goal() == False:
                    self.rotate_to_goal()

                self.move_forward()
            """
            while not self.is_goal_reached() and not self.obstacle_found:

                if self.is_towards_goal() == False:
                    self.rotate_to_goal()

                self.move_forward()

            if self.obstacle_found:
                self.stop()
                self.obstacle_circumvent()

                # self.bug()

                # self.take_actions()
                time.sleep(2)

                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                time.sleep(4)

                # self.obstacle_circumvent()

            """
            if self.new_obstacle():
                self.stop()
                #self.obstacle_found = False
                self.take_actions()
            else:
                print("Goal is reached.")
            """

            """
            while self.is_goal_reached() == False:
                self.take_actions()

            print("Goal is reached.")
            """
        print("Goal is reached.")

        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def bug(self):
        while abs(self.curr_line - self.mline) > 0.1:
            # print(abs(self.curr_line - self.mline))
            self.vel_msg.linear.x = 0.1
            if self.regions['left'] < OBSTACLE_THRESHOLD - 0.2:  # too close to obstacle, rotate away
                # LEFT90 = 600
                self.vel_msg.angular.z = -0.1
                print("BUG2")
            elif self.regions['fright'] > OBSTACLE_THRESHOLD + 0.2:  # too far, move closer
                # LEFT90 = 550
                self.vel_msg.angular.z = 0.3
                print("BUG3")
            else:
                self.vel_msg.angular.z = 0.0
                print("BUG4")

            self.velocity_publisher.publish(self.vel_msg)
            # time.sleep(2)
        print("bgika apo while")

    def obstacle_circumvent(self):

        while self.take_actions() == True:
            # self.take_actions()
            print()
        # time.sleep(2)

        self.stop()
        """
        while abs(self.curr_line - self.mline) > 0.1:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.move_forward()
        """

        self.move_forward()

        # hreiazetai auto?
        """
        while self.regions['front'] > 1 and self.regions['fleft'] > 1 and self.regions['fright'] < 1 or self.regions['front'] > 1 and self.regions['fleft'] < 1 and self.regions['fright'] > 1:
            self.move_forward()
            print("BHKAAAAAAAAA")
        """

    def take_actions(self):
        self.vel_msg = Twist()
        # linear_x = 0
        # angular_z = 0

        flag = False

        state_description = ''

        if self.regions['front'] > 0.4 and self.regions['fleft'] > 0.4 and self.regions['fright'] > 0.4:
            state_description = 'case 1 - nothin'

            print(state_description)
            linear_x = 0.2
            angular_z = 0

        elif self.regions['front'] < 0.4 and self.regions['fleft'] > 0.4 and self.regions['fright'] > 0.4:
            state_description = 'case 2 - front'
            print(state_description)

            linear_x = 0
            angular_z = 0.2

            flag = True

        elif self.regions['front'] > 0.4 and self.regions['fleft'] > 0.4 and self.regions['fright'] < 0.4:
            state_description = 'case 3 - fright'
            print(state_description)

            # to ehw peiraksei

            linear_x = 0.2
            angular_z = 0

            flag = True

        elif self.regions['front'] > 0.4 and self.regions['fleft'] < 0.4 and self.regions['fright'] > 0.4:
            state_description = 'case 4 - fleft'
            print(state_description)

            # to ehw peiraksei

            linear_x = 0.2
            angular_z = 0

            flag = True

        elif self.regions['front'] < 0.4 and self.regions['fleft'] > 0.4 and self.regions['fright'] < 0.4:
            state_description = 'case 5 - front and fright'
            print(state_description)

            linear_x = 0
            angular_z = 0.2

            flag = True

        elif self.regions['front'] < 0.4 and self.regions['fleft'] < 0.4 and self.regions['fright'] > 0.4:
            state_description = 'case 6 - front and fleft'
            print(state_description)

            linear_x = 0
            angular_z = -0.2

            flag = True

        elif self.regions['front'] < 0.4 and self.regions['fleft'] < 0.4 and self.regions['fright'] < 0.4:
            state_description = 'case 7 - front and fleft and fright'
            print(state_description)

            linear_x = 0
            angular_z = 0.2

            flag = True

        elif self.regions['front'] > 0.4 and self.regions['fleft'] < 0.4 and self.regions['fright'] < 0.4:
            state_description = 'case 8 - fleft and fright'
            print(state_description)

            linear_x = 0.2
            angular_z = 0

            flag = True

        else:
            state_description = 'unknown case'
            print(state_description)

            rospy.loginfo(self.regions)

        # shmainei oti apefuga to empodio
        # kanw epanaupologismo diadromis
        """
        if self.obstacle_circumventing == True:
            print("HI")
            self.start = (self.pos.position.x, self.pos.position.y)
            self.rotate_to_goal()
            self.obstacle_circumventing = False
        """

        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z
        self.velocity_publisher.publish(self.vel_msg)

        return flag

    def move_forward(self):
        # print(">>>>> Move forward")
        self.vel_msg.linear.x = 0.2
        self.velocity_publisher.publish(self.vel_msg)
        return

    def is_goal_reached(self):  # Check if goal is reached.
        if (self.x_goal - 0.2 < x_robot < self.x_goal + 0.2) and (self.y_goal - 0.2 < y_robot < self.y_goal + 0.2):
            return True
        return False

    def rotate_to_goal(self):  # Rotate towards goal.
        # print(">>>>> Rotate towards goal")
        global yaw, vel_msg
        desired_angle_goal = math.atan2(self.y_goal - y_robot, self.x_goal - x_robot)
        K_angular = 0.5
        angular_speed = (desired_angle_goal - yaw) * K_angular
        while True:
            vel_msg.angular.z = angular_speed
            self.velocity_publisher.publish(vel_msg)

            # if desired_angle_goal < 0:
            if abs((desired_angle_goal) - yaw) < 0.3:
                break

            """
            if desired_angle_goal < 0:
                if ((desired_angle_goal) - yaw) > -0.1:
                    break
            else:
                if ((desired_angle_goal) - yaw) < 0.1:
                    break
            """
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    # tupika de hreiazetai an allaksw ligo to rotate_to_goal
    def is_towards_goal(self):  # Check if robot in direction of the goal.
        desired_angle_goal = math.atan2(self.y_goal - y_robot, self.x_goal - x_robot)
        if abs((desired_angle_goal) - yaw) < 0.3:
            return True
        return False

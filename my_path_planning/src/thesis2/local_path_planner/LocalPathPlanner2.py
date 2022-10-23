#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovarianceStamped
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
import time
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


# Globals
x_robot = 0
y_robot = 0
yaw = 0
vel_msg = Twist()
TOLERANCE = 0.5


class LocalPathPlanner:
    def __init__(self, points):

        self.gesture_flag_terminate = 0
        rospy.Subscriber("/gesture_class", String, self.gesture_callback)
        rospy.sleep(5)
        self.points = points

        #self.velocity_publisher = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)



        #rospy.Subscriber("/odom", Odometry, self.pose_callback)
        #rospy.Subscriber("/RosAria/pose", Odometry, self.pose_callback)

        #rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        self.path_visualization = Path()
        self.path_pub = rospy.Publisher('/trajectory', Path, queue_size=10)

        #FOR PATH VISUALIZATION PURPOSE
        #rospy.Subscriber("/odom", Odometry, self.odom_cb_path)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.robot_trajectory)

        #TRANSFORMS
        rospy.Subscriber("/robot_pose", Float64MultiArray, self.robot_pose_tf)
        rospy.Subscriber("/robot_orientation", Float64MultiArray, self.robot_orientation_tf)


        self.pos = Pose()
        self.goal = None
        self.x_goal = None
        self.y_goal = None
        self.sonar_data = []
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.obstacle_found = False
        self.obstacle_circumventing = None

        self.regions = None



        self.point_changed = 0

        self.direction = None
        #self.laser_distance_value = 0.3 #gia to simulation 0.3
        self.laser_distance_value = 0.7 # einai kali h timi auti???????????

        # rospy.Subscriber("/base_scan", LaserScan, self.clbk_laser)
        rospy.Subscriber("/scan", LaserScan, self.clbk_laser)




    def pose_callback(self, pose_data):
        global x_robot, y_robot, yaw
        
        self.pos = pose_data.pose.pose.position
        
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
    def pose_callback(self, amcl_pose):
        global x_robot, y_robot, yaw

        self.pos = amcl_pose.pose.pose.position

        x_robot = amcl_pose.pose.pose.position.x
        y_robot = amcl_pose.pose.pose.position.y

        quaternion = (
            amcl_pose.pose.pose.orientation.x,
            amcl_pose.pose.pose.orientation.y,
            amcl_pose.pose.pose.orientation.z,
            amcl_pose.pose.pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        yaw = rpy[2]
    """

    # NEW ADDED!!!!!!!!!!!!!!!!!!!
    def robot_pose_tf(self, robot_pose):
        global x_robot, y_robot
        self.pos = robot_pose
        x_robot = robot_pose.data[0]
        y_robot = robot_pose.data[1]


    def robot_orientation_tf(self, robot_orientation):
        global yaw
        quaternion = (
            robot_orientation.data[0],
            robot_orientation.data[1],
            robot_orientation.data[2],
            robot_orientation.data[3])

        rpy = tf.transformations.euler_from_quaternion(quaternion)
        yaw = rpy[2]

    """
    def odom_cb_path(self, data):
        self.path_visualization.header = data.header

        self.pose = PoseStamped()
        self.pose.header = data.header
        self.pose.pose = data.pose.pose
        #print(self.pose)
        self.path_visualization.poses.append(self.pose)
        self.path_pub.publish(self.path_visualization)
    """

    def robot_trajectory(self, amcl_pose):
        self.path_visualization.header = amcl_pose.header

        self.pose = PoseStamped()
        self.pose.header = amcl_pose.header
        self.pose.pose = amcl_pose.pose.pose
        #print(self.pose)
        self.path_visualization.poses.append(self.pose)
        self.path_pub.publish(self.path_visualization)

    def gesture_callback(self, gesture_class):
        self.gesture = gesture_class.data
        #print(self.gesture)
        if self.gesture == 'six':
            #print("SIX SIX SIX")

            self.gesture_flag_terminate = 1
            return
        # print(self.gesture)

    def clbk_laser(self, msg):
        #print(len(msg.ranges))


        #REAL ROBOT (1)
        # 180 / 5 = 36
        regions = {
            'right': min(min(msg.ranges[0:35]), 10),
            'fright': min(min(msg.ranges[36:71]), 10),
            'front': min(min(msg.ranges[72:107]), 10),
            'fleft': min(min(msg.ranges[108:143]), 10),
            'left': min(min(msg.ranges[144:179]), 10),
        }
        #minval = min(msg.ranges[288:431])
        #print(minval)
        self.regions = regions


        """
        #SIMULATION
        #len(ranges) = 720 kai ehw times apo -180 mehri 180.
        #thelw na diairesw/5 to sunolo timwn (180-539)
        #ara 360/5 = 72
        regions = {
            'right': min(min(msg.ranges[180:251]), 10),
            'fright': min(min(msg.ranges[252:323]), 10),
            'front': min(min(msg.ranges[324:395]), 10),
            'fleft': min(min(msg.ranges[396:467]), 10),
            'left': min(min(msg.ranges[468:539]), 10),
        }
        #minval = min(msg.ranges[288:431])
        #print(minval)
        self.regions = regions

        """


        #TODO na allaksw tis times gia obstacle detection

        # OBSTACLE DETECTION
        #evgala tous elegxous gia right kai left giati ousiastika den me endiaferoun autes oi periptwseis kai den kanw kati gia autes parakatw
        if self.regions['fright'] < self.laser_distance_value or self.regions['front'] < self.laser_distance_value or self.regions['fleft'] < self.laser_distance_value:
            self.obstacle_found = True
        else:
            self.obstacle_found = False
        #print(self.obstacle_found)



    def stop(self):
        """
        Method to bring robot to a halt by publishing linear and angular velocities of zero.
        """
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def go(self):
        """
        Method to go to goal specified irrespective of the current position and orientation
        of the robot.
        """
        time.sleep(2.0)

        print("Navigation for point (3,3) has started.")

        #!!!!!!!!!!!!!!!!!!!!!!!!
        min_dist = math.inf
        min_dist_pos = None
        if self.gesture_flag_terminate == 1:
            print("Halt gesture recognised.")
            return

            # ta points einai se world coordinates
            # phgainei apo shmeio se shmeio
        for index, goal in enumerate(self.points):
            self.point_changed = 1
            # tin twrini thesi tin pairnw apo to callback

            self.goal = goal

            self.x_goal = goal.x
            self.y_goal = goal.y

            #print(self.euclidean_distance(goal))

            while not self.is_goal_reached() and self.gesture_flag_terminate != 1:
                self.point_changed = 0
                #print("edw kollisa 3")
                if self.obstacle_found:
                    #print("edw kollisa 2")
                    self.stop()
                    if self.gesture_flag_terminate == 1:
                        print("Halt gesture recognised.")
                        return
                    """
                    #elegxos gia tis oriakes times, hreiazetai logw tou heirismou twn grid cells
                    if (self.x_goal - self.laser_distance_value - 0.2 < x_robot < self.x_goal + self.laser_distance_value + 0.2) and (self.y_goal - self.laser_distance_value - 0.2 < y_robot < self.y_goal + self.laser_distance_value + 0.2):
                        print("oriaki timi")
                        break
                    """

                    self.avoid_obstacle()
                    #self.move_forward()

                    time.sleep(2)
                    print(index)


                    if index == 0:
                        break

                    elif index == len(self.points) - 1:
                        break
                        """
                        if counter == - 1:
                            continue

                        elif counter == 0:
                            #start_pos_x_obstacle_avoidance = x_robot
                            #start_pos_y_obstacle_avoidance = y_robot
                            start_pos_obstacle_avoidance = self.pos


                        temp_dist = self.euclidean_distance(self.points[index])

                        if temp_dist < min_dist:
                            min_dist = temp_dist
                            min_dist_pos = self.pos

                        #ousiastika oso den ehei ginei goal is reached, tha prospathei na proseggisei to goal

                        #estw oti de borei na bei se loop kai na phgainei bros-pisw stis theseis
                        if self.euclidean_distance(start_pos_obstacle_avoidance) < TOLERANCE and counter > 9:  # thelw na epivevaiwsw oti ehw kanei kapoio kuklo gurw apo to obstacle
                            #tha prosthesw to min_dist shmeio ston pinaka me ta goals kai tha paw ekei
                            self.points.append(min_dist_pos)
                            counter = -1
                            break

                        counter += 1

                        """
                    elif self.euclidean_distance(self.points[index - 1]) > self.euclidean_distance(self.points[index + 1]):
                        break

                else:
                    if not self.is_towards_goal():
                        if self.gesture_flag_terminate == 1:
                            print("Halt gesture recognised.")
                            return
                        self.rotate_to_goal()
                        #print("edw kollisa 1")
                    self.move_forward()


            if self.is_goal_reached():
                print("Goal is reached.")
            else:
                print("Moving to next goal because of the obstacle.")

        self.stop()
        # time.sleep(2)

    def avoid_obstacle(self):
        #print("Edw kollisa 5")

        while self.take_actions():  #OSO EPISTREFEI TRUE
            if (self.x_goal - 0.5 < x_robot < self.x_goal + 0.5) and (self.y_goal - 0.5 < y_robot < self.y_goal + 0.5):
                break

            #print("Edw kollisa 6")

            #time.sleep(0.1)
        # time.sleep(2)

        #self.stop()
        #self.move_forward()
        self.follow_obstacle()

        #time.sleep(2)
        self.stop()

    def take_actions(self):
        #self.vel_msg = Twist()
        linear_x = 0
        angular_z = 0

        #laser_distance_value = 0.4
        flag = False
        state_description = ''
        #TODO isws hreiastei na allaksw tis times
        #print(self.gesture_flag_terminate)
        if self.gesture_flag_terminate == 1:
            print("Halt gesture recognised.")
            return

        if self.regions['front'] > self.laser_distance_value and self.regions['fleft'] > self.laser_distance_value and self.regions['fright'] > self.laser_distance_value:
            state_description = 'case 1 - nothin'
            #print("Edw kollisa 4")
            #linear_x = 0.3
            linear_x = 0
            angular_z = 0

        elif self.regions['front'] < self.laser_distance_value and self.regions['fleft'] > self.laser_distance_value and self.regions['fright'] > self.laser_distance_value:
            state_description = 'case 2 - front'

            linear_x = 0
            angular_z = 0.1

            if self.point_changed == 0:
                if self.direction != 'fright':
                    self.direction = 'fleft'
            #self.direction = 'fright'

            flag = True

        elif self.regions['front'] > self.laser_distance_value and self.regions['fleft'] > self.laser_distance_value and self.regions['fright'] < self.laser_distance_value:
            state_description = 'case 3 - fright'

            # to ehw peiraksei

            #linear_x = 0.2
            #angular_z = 0

            linear_x = 0
            angular_z = 0.1
            self.direction = 'fright'

            flag = True

        elif self.regions['front'] > self.laser_distance_value and self.regions['fleft'] < self.laser_distance_value and self.regions['fright'] > self.laser_distance_value:
            state_description = 'case 4 - fleft'

            # to ehw peiraksei

            #linear_x = 0.2
            #angular_z = 0

            linear_x = 0
            angular_z = -0.1
            self.direction = 'fleft'

            flag = True

        elif self.regions['front'] < self.laser_distance_value and self.regions['fleft'] > self.laser_distance_value and self.regions['fright'] < self.laser_distance_value:
            state_description = 'case 5 - front and fright'

            #linear_x = -0.1
            #angular_z = 0.2
            linear_x = 0
            angular_z = 0.1
            self.direction = 'fright'
            flag = True

        elif self.regions['front'] < self.laser_distance_value and self.regions['fleft'] < self.laser_distance_value and self.regions['fright'] > self.laser_distance_value:
            state_description = 'case 6 - front and fleft'

            #linear_x = -0.1
            #angular_z = -0.2

            linear_x = 0
            angular_z = -0.1
            self.direction = 'fleft'
            flag = True

        elif self.regions['front'] < self.laser_distance_value and self.regions['fleft'] < self.laser_distance_value and self.regions['fright'] < self.laser_distance_value:
            state_description = 'case 7 - front and fleft and fright'

            linear_x = 0
            angular_z = 0.1
            if self.point_changed == 0:
                if self.direction != 'fright':
                    self.direction = 'fleft'

            flag = True

        #TODO na elegksw an dimiourgei provlima
        elif self.regions['front'] > self.laser_distance_value and self.regions['fleft'] < self.laser_distance_value and self.regions['fright'] < self.laser_distance_value:
            state_description = 'case 8 - fleft and fright'

            linear_x = 0.2
            angular_z = 0
            self.direction = 'fright'
            flag = True

        """
        #de hreiazetai
        else:
            state_description = 'unknown case'
            #print(state_description)

            rospy.loginfo(self.regions)
        """

        print(state_description)

        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z
        self.velocity_publisher.publish(self.vel_msg)
        rospy.sleep(0.1)
        self.vel_msg.linear.x = -0.01    #NEW ADDED!!!!!!!!!!!!!!!!!!!!
        self.vel_msg.angular.z = 0
        while self.regions['front'] < 0.2:
            self.velocity_publisher.publish(self.vel_msg)
        #rospy.sleep(0.01)
        return flag

    def follow_obstacle(self):
        #current_pos = self.pos.data
        current_pos = Point()
        current_pos.x = self.pos.data[0]
        current_pos.y = self.pos.data[1]

        #ehei provlima giati ti mia hrisimopoiw stin euclidean distance ta nodes, kai pairnw tis times node[x]
        #enw edw einai pinakas, den exei x,y pedia.
        #current_pos = (self.pos[0], self.pos[1])
        current_orientation = yaw
        #TODO TI TIMES NA VALW EDW GIA TIN APOSTASI


        #TODO NA KSANADW TIS TIMES ELEGXOU

        #self.vel_msg.linear.x = 0.2
        counter = 0
        flag = 0
        # EGW OUSIASTIKA NOMIZW OTI THELW: KINHSI MEHRI NA ALLAKSEI O PROSANATOLISMOS
        # KAI OTAN ALLAKSEI LIGI AKOMA KINISI GIA NA MHN PESW SE LOOP BROS PISW

        #while(self.euclidean_distance(current_pos) <= 0.5 and abs((current_orientation - yaw) < 0.5)):
        while((self.euclidean_distance(current_pos) <= 1.5 ) and abs((current_orientation - yaw) < 0.5)):
            if self.gesture_flag_terminate == 1:
                print("Halt gesture recognised.")
                return
            #print("Edw kollisa 7")
            while self.regions['front'] < 0.2:
                self.vel_msg.linear.x = -0.1
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                if (self.x_goal - 0.5 < x_robot < self.x_goal + 0.5) and (self.y_goal - 0.5 < y_robot < self.y_goal + 0.5):
                    flag = 1
            rospy.sleep(0.01)


            self.vel_msg.linear.x = 0.2
            #print("HI")
            print(self.direction)
            if self.direction == 'fright':
                #print("KOLLISE FRIGHT")
                if self.regions['fright'] > 0.7: # too far, move closer
                    self.vel_msg.angular.z = -0.1
                    #print("too far, move closer")

                elif self.regions['fright'] <= 0.7: # too close to obstacle, rotate away
                    self.vel_msg.angular.z = 0.2
                    #print("too close to obstacle, rotate away")


            elif self.direction == 'fleft':
                #print("KOLLISE FLEFT")
                if self.regions['fleft'] > 0.7:  # too far, move closer
                    self.vel_msg.angular.z = 0.1
                    #print("too far, move closer")

                elif self.regions['fleft'] <= 0.7:  # too close to obstacle, rotate away
                    self.vel_msg.angular.z = -0.2
                    #print("too close to obstacle, rotate away")

            counter += 1
            self.velocity_publisher.publish(self.vel_msg)

            time.sleep(0.1)

            if flag == 1: #oriaki timi se gwnia pou duskolevetai na akolouthisei
                break
        #print(counter)
        # AUTO TO EVALA SE PERIPTWSI POU VGEI APO TI WHILE KAI EINAI KONTA STO OBSTACLE
        # NA MIN KANEI AMESWS PALI OBSTACLE FOUND EPEIDI THA EINAI KONTA STO EMPODIO
        # ALLA NA PROSPATHISEI EK NEOU NA PROSEGGISEI TO SHMEIO
        if self.direction == 'fright':
            self.vel_msg.angular.z = 0.1
        if self.direction == 'fleft':
            self.vel_msg.angular.z = -0.1

        self.velocity_publisher.publish(self.vel_msg)
        #time.sleep(1)

    def move_forward(self):
        # print(">>>>> Move forward")
        self.vel_msg.linear.x = 0.3
        self.velocity_publisher.publish(self.vel_msg)
        return

    #to tolerance hreiazetai giati borei to robot na prospathei sunehws na paei akribws sto simeio
    #kai logw twn tahutitwn kai twn elegxwn na apehei ligo diarkws
    def is_goal_reached(self):  # Check if goal is reached.
        if (self.x_goal - 0.2 < x_robot < self.x_goal + 0.2) and (self.y_goal - 0.2 < y_robot < self.y_goal + 0.2):
            return True
        return False

    def rotate_to_goal(self):  # Rotate towards goal.
        # print(">>>>> Rotate towards goal")
        global yaw, vel_msg
        desired_angle_goal = math.atan2(self.y_goal - y_robot, self.x_goal - x_robot)
        K_angular = 0.2
        angular_speed = (desired_angle_goal - yaw) * K_angular
        while True:
            vel_msg.angular.z = angular_speed
            self.velocity_publisher.publish(vel_msg)
            # if desired_angle_goal < 0:
            if abs((desired_angle_goal) - yaw) < 0.3:
                break

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def is_towards_goal(self):  # Check if robot in direction of the goal.
        desired_angle_goal = math.atan2(self.y_goal - y_robot, self.x_goal - x_robot)
        if abs((desired_angle_goal) - yaw) < 0.3:
            return True
        return False

    def euclidean_distance(self, goal):
        """
        Method to compute distance from current position to the goal
        @returns 	euclidean distance from current point to goal
        """
        return math.sqrt(
            math.pow((goal.x - x_robot), 2) + math.pow((goal.y - y_robot), 2))
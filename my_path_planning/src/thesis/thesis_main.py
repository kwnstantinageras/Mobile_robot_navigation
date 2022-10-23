#!/usr/bin/env python
import math

from global_path_planner.GlobalPathPlanner import GlobalPathPlanner
from local_path_planner.LocalPathPlanner import LocalPathPlanner
from global_path_planner.Node import Node


def main():
    """
        1. kalw to programma tis anagnwrisis

        2. mou epistrefei poia heironomia einai kai analoga kalw ton global path planner pou mou epistrefei to global path se points
        3.me ta points auta paw sto local path planner.

    """
    points = []

    # tha prepei na metatrepw tin arhiki thesi se world coordinates
    # h thesi apo tin odometria einai vasei twn world coordinates?
    # tha pairnw sigoura pliroforia apo to odom gia ti thesi

    #start = Node(-12.0, 12.0, math.pi)
    #goal = Node(-15.0, 12.0, math.pi)

    # to theta den tha hreiastei kan logika sto global planning
    # tha prepei na to dinw sto local

    # ti thesi gia to start tha prepei na tin pairnw apo to amcl
    global_path = GlobalPathPlanner(start, goal)
    # einai pio swsto na to valw ston constructor?
    global_path.load_map('/home/konstantina/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/map/small_world.pgm')
    points = global_path.plan()

    # run local path planner on the translated coordinates


    local_path = LocalPathPlanner(points)
    local_path.go()


if __name__ == "__main__":
    main()

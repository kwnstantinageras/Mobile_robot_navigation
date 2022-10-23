#!/usr/bin/env python
import math
import rospy



MOVES = [(0, 0.5), (0, -0.5), (0.5, 0), (-0.5, 0)]
MOVES2 = [(0, 1), (0, -1), (1, 0), (-1, 0)]

def dijkstra(nodes, start, goal):
    current_node = None
    visited_set = []

    unvisited_set = nodes


    for node in nodes:
        if node.x == start.x and node.y == start.y:
            node.weight = 0



    while len(unvisited_set) != 0:
        min_distance = math.inf
        u = None
        foundMIN = 0
        #print(len(unvisited_set))

        # vriskw to node me to mikrotero varos
        for node in unvisited_set:

            if node.weight < min_distance:
                min_distance = node.weight
                u = node
                foundMIN = 1
                #print("EFTASA??????")
                #print(" eftasa???? %s , %s" % (u.x, u.y))

        """
        print("Unvisited node set: ")
        for node in unvisited_set:
            print("%s , %s, %s" % (node.x, node.y, node.weight))
        #rospy.sleep(2)
        print("Node with min weight is %s, %s" % (u.x, u.y) )
        rospy.sleep(2)
        """
        unvisited_set.remove(u)
        visited_set.append(u)

        #NEW ADDED!!!!!!!!!!!!
        if u.x == goal[0] and u.y == goal[1]:
            break

        flag = 0
        # for each neighbor of u

        for move in MOVES:
            neighbor = ((u.x + move[0]), (u.y + move[1]))

            for node in unvisited_set:
                if node.x == neighbor[0] and node.y == neighbor[1]:
                    temp_distance = u.weight + 0.5
                    if temp_distance < node.weight:
                        node.weight = temp_distance
                        node.parent = u


        for move in MOVES2:
            neighbor = ((u.x + move[0]), (u.y + move[1]))

            for node in unvisited_set:
                print(neighbor[0], neighbor[1])
                if node.x == neighbor[0] and node.y == neighbor[1]:
                    temp_distance = u.weight + 1
                    if temp_distance < node.weight:
                        node.weight = temp_distance
                        node.parent = u


    """
    for node in visited_set:

        print("%s , %s" % (node.x, node.y))
    """

    for node in visited_set:
        if node.x == goal[0] and node.y == goal[1]:
            current_node = node
            break

    path = []  # path needs to be in world coordinates
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent

    path.reverse()
    path = path[1:]
    # path.append(self.goal)

    for node in path:
        print("%s , %s" % (node.x, node.y))

    return path
#!/usr/bin/env python


import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from global_path_planner.NodeDijkstra import Node


class GridController:
    def __init__(self):
        self.unoccupied_cells = []
        self.unoccupied_cells_world_coordinates = []
        #rospy.init_node('grid_controller', anonymous=True)




        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.map_resolution = None

        self.occupancy_grid_data = None

        self.occupancy_grid_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, occupancy_grid):
        self.map_width = occupancy_grid.info.width
        self.map_height = occupancy_grid.info.height
        self.map_origin = occupancy_grid.info.origin
        self.map_resolution = occupancy_grid.info.resolution

        self.occupancy_grid_data = occupancy_grid.data

        #self.occupancy_grid_subscriber.shutdown()

    # returns graph nodes
    def unoccupied_cells_from_grid(self):

        while(self.map_height == None):
            rospy.sleep(0.1)


        for i in range(self.map_height):
            for j in range(self.map_width):
                new = 1
                current_cell_value = self.occupancy_grid_data[i + (self.map_width * j)]

                if current_cell_value == 0:
                    point = self.cell_to_point(i, j)
                    #print("Cell: %s, %s" % (i, j))
                    #print("Point: %s" % point)
                    new_node = Node(point[0], point[1])

                    for node in self.unoccupied_cells_world_coordinates:
                        if node.x == new_node.x and node.y == new_node.y:
                            new = 0

                    if new == 1:
                        self.unoccupied_cells_world_coordinates.append(new_node)
                        #print("x: %s" % new_node.x)
                        #print("y: %s" % new_node.y)
                        self.unoccupied_cells.append((i, j))
                    # self.cell_to_point(i, j)


    def cell_to_point(self, i, j):

        map_x_coordinate_in_meters = (i * self.map_resolution) + self.map_origin.position.x + self.map_resolution / 2
        map_y_coordinate_in_meters = (j * self.map_resolution) + self.map_origin.position.y + self.map_resolution / 2

        return [map_x_coordinate_in_meters, map_y_coordinate_in_meters]



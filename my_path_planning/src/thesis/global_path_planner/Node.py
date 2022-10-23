#!/usr/bin/env python

"""
Dynamically generate graph nodes from current location
"""

import math

SIMILARITY_THRESHOLD = 0.1
SAFETY_OFFSET = 5  # number of pixels away from the wall the robot should remain

MAP_WIDTH = 125.0
MAP_HEIGHT = 43.75
LASER_MAX = 8.0


class Node:
    def __init__(self, x, y, theta=0.0):
        self.x = x
        self.y = y
        # den kserw an hreiazetai to theta
        self.theta = theta
        self.parent = None
        self.weight = math.inf

    def is_similar(self, other):
        """
        Return true if other node is in similar position as current node
        """
        return self.euclidean_distance(other) <= SIMILARITY_THRESHOLD

    def is_valid(self, grid_map):
        """
        Return true if the location on the map is valid, ie, in obstacle free zone

        goal_pixel = self.world_to_pixel((self.x, self.y), (700, 2000))
        if grid_map[goal_pixel[0]][goal_pixel[1]]:
            return True
        return False
        """
        # image size ??????????????????????????????????????????????????????
        goal_pixel = self.world_to_pixel((self.x, self.y), (700, 2000))
        if grid_map[goal_pixel[0]][goal_pixel[1]] != 0:
            return True
        return False

    def is_move_valid(self, grid_map, move):
        """
        Return true if required move is legal
        """
        goal = self.apply_move(move)
        # convert goal coordinates to pixel coordinates before checking this
        goal_pixel = self.world_to_pixel((goal.x, goal.y), (700, 2000))
        # check if too close to the walls
        if goal_pixel[0] >= SAFETY_OFFSET and not grid_map[goal_pixel[0] - SAFETY_OFFSET][goal_pixel[1]]:
            return False
        if goal_pixel[1] >= SAFETY_OFFSET and not grid_map[goal_pixel[0]][goal_pixel[1] - SAFETY_OFFSET]:
            return False
        if goal_pixel[0] >= SAFETY_OFFSET and goal_pixel[1] >= SAFETY_OFFSET and not \
                grid_map[goal_pixel[0] - SAFETY_OFFSET][goal_pixel[1] - SAFETY_OFFSET]:
            return False
        if grid_map[goal_pixel[0]][goal_pixel[1]]:
            return True
        return False

    def apply_move(self, move):
        """
        Apply the given move to current position
        @arg 	move 	[length, dtheta]
        """
        theta_new = self.theta + move[1]
        x_new = self.x + math.cos(theta_new) * move[0]  # d.cos(theta)
        y_new = self.y + math.sin(theta_new) * move[0]  # d.sin(theta)
        return Node(x_new, y_new, theta_new)

    def euclidean_distance(self, goal):
        """
        Method to compute distance from current position to the goal
        @arg	goal 	Node object with x, y, theta
        @returns 	euclidean distance from current point to goal
        """
        return math.sqrt(math.pow((goal.x - self.x), 2) + math.pow((goal.y - self.y), 2))

    # TO-D0 na tsekarw ti vgazei h synartisi
    # oi upologismoi autoi einai swstoi?
def world_to_pixel(self, world_points, image_size):
    world_x, world_y = world_points
    img_h, img_w = image_size
    pixel_points = []
    pixel_points[0] = int(max((world_x / MAP_WIDTH) * img_w, 0))
    if pixel_points[0] > img_w - 1:
        pixel_points[0] = img_w - 1
    pixel_points[1] = int(max((world_y / MAP_HEIGHT) * img_h, 0))
    if pixel_points[1] > img_h - 1:
        pixel_points[1] = img_h
    pixel_points[1] = pixel_points[1]
    pixel_points[0] = img_w / 2 + pixel_points[0]
    pixel_points[1] = img_h / 2 - pixel_points[1]
    return pixel_points

    # TO-D0 na tsekarw ti vgazei h synartisi
    # TO-DO na dw pws kai an borei na sunduastei me tin sunartisi is_similar
    # oi upologismoi autoi einai swstoi?
def pixel_to_world(self, pixel_points, image_size):
    img_h, img_w = image_size
    pixel_x, pixel_y = pixel_points
    world_points = [None] * 2
    world_points[0] = pixel_x / img_w * MAP_WIDTH
    world_points[1] = (pixel_y / img_h * MAP_HEIGHT)
    world_points[0] = world_points[0] - MAP_WIDTH / 2
    world_points[1] = world_points[0] + MAP_HEIGHT / 2
    return world_points

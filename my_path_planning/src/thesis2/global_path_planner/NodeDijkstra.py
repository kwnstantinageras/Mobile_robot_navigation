#!/usr/bin/env python

"""
Dynamically generate graph nodes from current location
"""

import math


# every occupancy grid cell that is considered as obstacle free is a graph node


class Node:
    def __init__(self, x, y):
        self.x = self.round_coordinates(x)
        self.y = self.round_coordinates(y)
        self.parent = None
        self.weight = math.inf

    def round_coordinates(self, temp_number):
        temp_number = round(temp_number, 1)

        fractional, whole = math.modf(temp_number)

        if fractional <= 0.3:
            if whole >= 0:
                final_number = math.floor(temp_number)

            else:
                final_number = math.ceil(temp_number)
        elif fractional >= 0.7:
            if whole >= 0:
                final_number = math.ceil(temp_number)
            else:
                final_number = math.floor(temp_number)
        else:
            final_number = whole + 0.5

        return final_number

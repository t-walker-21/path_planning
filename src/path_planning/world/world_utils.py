"""
Class to create virtual world 
"""

import cv2
import numpy as np

class World():
    def __init__(self, size, name="World"):
        """

        Initialize empty world
        """

        self.world = np.ones(np.append(size, 3)) * 255
        self.name = name
        self.size = size

    def show_world(self, wait):
        """

        Display the world
        """

        cv2.imshow(self.name, self.world)
        cv2.waitKey(wait)

    def add_obstacle(self, position, size, shape, color=(0, 0, 0)):
        """

        Add obstacles to world
        """

        if shape == "circle":

            # Add circle to map
            cv2.circle(self.world, position, size, color, -1) 

        elif shape == "rect":

            #Add rectangle to map
            #img, pt1, pt2, color, thickness=1, lineType=8, shift=0
            cv2.rectangle(self.world, position[0], position[1], color, -1)

    def add_random_obstacle(self):
        """

        Add a random obstacle to the world
        """

        x_lim = self.size[0] - 100
        y_lim = self.size[1] - 100
        radius_lim = 50

        pos_x = np.random.randint(100, x_lim)
        pos_y = np.random.randint(100, y_lim)

        size = np.random.randint(20, radius_lim)

        self.add_obstacle((pos_x, pos_y), size, "circle")


    def color_node(self, node, color=(100, 0, 0), thickness=-1):
        """

        Set a node to a particular circle with color
        """
        cv2.circle(self.world, node, 3, color, thickness) 

    def draw_line(self, pt1, pt2, color=(0, 0, 0)):
        """

        Draw a line connecting two points
        """

        cv2.line(self.world, pt1, pt2, color)
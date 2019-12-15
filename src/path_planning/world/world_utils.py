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

        cv2.imshow(self.name, cv2.resize(self.world, (300, 300)))
        cv2.waitKey(wait)

    def add_obstacle(self, position, size, shape, color=(0, 0, 0)):
        """

        Add obstacles to world
        """

        if shape == "circle":

            # Add circle to map
            cv2.circle(self.world, position, size, color, -1) 

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


    def color_node(self, node, color):
        """

        Set a node (pixel) to a particular color
        """

        self.world[node[0]][node[1]][0] = color[0]
        self.world[node[0]][node[1]][1] = color[1]
        self.world[node[0]][node[1]][2] = color[2]
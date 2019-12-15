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

        self.world = np.ones(np.append(size, 3))
        self.name = name

    def show_world(self, wait):
        """
        Display the world
        """

        cv2.imshow(self.name, self.world)
        cv2.waitKey(wait)
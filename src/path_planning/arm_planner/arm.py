"""
A class for modelling a simple RR robot arm
"""

import numpy as np
import cv2

class RobotArm:

    def __init__(self, link_1_length=3, link_2_length=3, joint_1=0, joint_2=0):

        self.link_1_length = link_1_length
        self.link_2_length = link_2_length

        
        # Convention is: when both joints are 0, arm is lying on the x-axis
        self.joint_1 = joint_1 * np.pi/180
        self.joint_2 = joint_2 * np.pi/180
        
        # Number of points to represent 2D space occupied by robot
        self.num_points = 10

        self.calc_transformed_points()

    def set_joint_angles(self, joint_1, joint_2, convert_to_radians=True):
        
        if convert_to_radians:
            self.joint_1 = joint_1 * np.pi/180
            self.joint_2 = joint_2 * np.pi/180

        else:
            self.joint_1 = joint_1
            self.joint_2 = joint_2

        self.calc_transformed_points()

    def calc_transformed_points(self):
        self.link_1_origin_points = [[x * np.cos(self.joint_1), x * np.sin(self.joint_1)] for x in np.linspace(0, self.link_1_length, self.num_points)]
        self.link_2_origin_points = [[x * np.cos(self.joint_2 + self.joint_1) + self.link_1_origin_points[-1][0], x * np.sin(self.joint_2 + self.joint_1) + self.link_1_origin_points[-1][1]] for x in np.linspace(0, self.link_2_length, self.num_points)]

        self.link_1_origin_points = np.array(self.link_1_origin_points, dtype=np.int)
        self.link_2_origin_points = np.array(self.link_2_origin_points, dtype=np.int)

        return self.link_1_origin_points, self.link_2_origin_points

    def forward(self):
        """
        Using current joint setting, return x,y position of EE
        """

        return int(self.link_1_length * np.cos(self.joint_1) + self.link_2_length * np.cos(self.joint_2 + self.joint_1)), int(self.link_1_length * np.sin(self.joint_1) + self.link_2_length * np.sin(self.joint_2 + self.joint_1))

    def backward(self, ee_x, ee_y, x_offset=350, y_offset=350, joint_2_mult=1):
        """
        Compute joint angles to satisfy x,y (arm frame points)
        """
        
        #cos_joint_2 = (ee_x**2 + ee_y**2 - self.link_1_length**2 - self.link_2_length**2) / (2 * self.link_2_length * self.link_1_length)
        joint_2 = joint_2_mult*np.arccos((ee_x**2 + ee_y**2 - self.link_1_length**2 - self.link_2_length**2) / (2 * self.link_2_length * self.link_1_length))
        joint_1 = np.arctan(ee_y / ee_x) - np.arctan((self.link_2_length * np.sin(joint_2))/(self.link_1_length + self.link_2_length*np.cos(joint_2)))

        return joint_1, joint_2

    def convert_to_world_frame(self, x, y, world_size_x=350, world_size_y=350):
        """
        Transform point in robot arm frame to world frame
        """

        return x + world_size_x, y + world_size_y

    def convert_to_arm_frame(self, x, y, world_size_x=350, world_size_y=350):
        """
        Transform point in world frame to arm frame
        """

        return x - world_size_x, y - world_size_y
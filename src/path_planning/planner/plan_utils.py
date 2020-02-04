"""

Utilities for path planning
"""
import numpy as np
from graph.grah_utils import Graph, graph_search, graph_search_recursive
import time
import cv2

"""

Class for RRT path planning
"""
class Planner(object):
    def __init__(self, world, gamma=0.1, iters=15000, reach=10, threshold=7):
        self.world = world.world.copy()
        self.world_h = world
        self.iters = iters
        self.gamma = gamma
        self.graph = Graph()
        self.reach = reach
        self.threshold = threshold

    def find_path(self, start, goal, visualize=False):
        """

        Given a start and goal node, compute a goal
        """

        # Add start and goal node to world

        self.world_h.color_node(start, color=(0, 0, 100), thickness=5)
        self.world_h.color_node(goal, color=(0, 100, 0), thickness=5)

        # Add start node to graph

        self.graph.add_vertex(start)

        for _ in range(self.iters):
            # Get a random point

            pt = self.get_random_point(self.world, goal)

            # If the graph is not empty, find the closest node in the graph and add it as an edge
            index = self.get_closest_point(pt)

            # Get vector pointing from index to current point

            vector = self.get_unit_vector(pt, index)

            new_node_pos =  tuple(vector + np.array(self.graph.vertices[index]))

            # If the new node is reachable, add it to the graph and world. Draw a line too

            if self.is_reachable(new_node_pos, index):
                self.graph.add_vertex(new_node_pos)
                self.graph.add_edge((self.graph.vertices[index], new_node_pos))

                self.world_h.color_node(new_node_pos)
                self.world_h.draw_line(new_node_pos, self.graph.vertices[index])


            else:
                #print "the new node could not be reached"
                self.world_h.color_node(new_node_pos, color=(0,10,10))
                

            # Check if new node is within a threshold distance to goal
            
            if (np.linalg.norm(np.array(new_node_pos) - np.array(goal)) <= self.threshold):
                print "Found a path!"
                self.world_h.show_world(0)
                self.graph.add_vertex(goal)
                self.graph.add_edge((new_node_pos, goal))

                #path = graph_search(self.graph, start, goal)
                path = []
                visited = []

                path = graph_search_recursive(self.graph, start, goal, path, visited)


                self.draw_final_path(path)
                self.world_h.show_world(0)

                break

            if visualize:
                self.world_h.show_world(visualize)

        

    def get_random_point(self, world, goal):
        """

        Get a random point within the bounds, based on gamma, sample goal point
        """

        chance = np.random.rand(1)

        if (self.gamma > chance):
            return goal

        # Get limits of world
        x_lim = world.shape[0] - 1
        y_lim = world.shape[1] - 1

        # Sample point
        point_x = np.random.randint(x_lim)
        point_y = np.random.randint(y_lim)

        return (point_x, point_y)


    def get_unit_vector(self, pt, index):
        vector = np.array(pt) - np.array(self.graph.vertices[index]) + np.array([1e-10, 1e-10])

        mag = np.linalg.norm(vector)

        return ((vector / mag) * self.reach).astype(np.int32)

    def is_reachable(self, start, goal):
        """

        Function to determine whether the vector b/w start and goal could be followed
        """

        if (self.window(start, win=21)):
            return False

        return True

    def draw_final_path(self, path):
        for node in path:
            self.world_h.color_node(node, color=(100, 0, 100))

    def window(self, center, win=11):
        """

        Place a window around the point and check for collison
        """

        width = int(win / 2.0)

        x_low = max(center[0] - width, 0)
        x_high = min(center[0] + width, 799)

        y_low = max(center[1] - width, 0)
        y_high = min(center[1] + width, 799)

        for i in range(x_low, x_high):
            for j in range(y_low, y_high):

                if ((self.world[j][i] == 0).all()):
                    return True

        

        return False


    def get_closest_point(self, node):
        """

        Calculate the closest node in the graph
        """

        distances = []

        for pt in self.graph.vertices:
            
            distances.append(np.linalg.norm(np.array(pt) - np.array(node)))

        return np.argmin(distances)
"""

Utilities for path planning
"""
import numpy as np
from graph.grah_utils import Graph

"""

Class for RRT path planning
"""
class Planner(object):
    def __init__(self, world, gamma=0.2, iters=1000, reach=2):
        self.world = world.world.copy()
        self.world_h = world
        self.iters = iters
        self.gamma = gamma
        self.graph = Graph()
        self.reach = reach

    def find_path(self, start, goal):
        """

        Given a start and goal node, compute a goal
        """

        # Add start and goal node to world

        self.world_h.color_node(start, thickness=5)
        self.world_h.color_node(goal, thickness=5)

        # Add start node to graph

        self.graph.add_vertex(start)

        for _ in range(self.iters):
            # Get a random point

            pt = self.get_random_point(self.world, goal)

            """
            # If the graph is empty, add this initial point and return
            if (len(self.graph.vertices) == 0):
                self.graph.add_vertex(pt)

                print (pt)

                self.world_h.color_node(pt)

                self.world_h.show_world(0)
                break"""

            # If the graph is not empty, find the closest node in the graph and add it as an edge
            index = self.get_closest_point(pt)

            # Get unit vector pointing from index to current point

            unit = self.get_unit_vector(pt, index)

            # Using the unit vector and reach, calculate position of new node

            new_node_pos = unit * self.reach

            # If the new node is reachable, add it to the graph and world. Draw a line too

            self.graph.add_vertex(new_node_pos)
            self.graph.add_edge((self.graph.vertices[index], new_node_pos))

    def get_random_point(self, world, goal):
        """

        Get a random point within the bounds, based on gamma, sample goal point
        """

        if (self.gamma > np.random.rand(1)):
            return goal

        # Get limits of world
        x_lim = world.shape[0] - 1
        y_lim = world.shape[1] - 1

        # Sample point
        point_x = np.random.randint(x_lim)
        point_y = np.random.randint(y_lim)

        return (point_x, point_y)


    def get_unit_vector(self, pt, index):
        pass

    def draw_line(self, pt1, pt2, color=(0, 0, 0)):
        """

        Draw a line connecting two points
        """

        cv2.line(self.world, pt1, pt2, color)

    def get_closest_point(self, node):
        """

        Calculate the closest node in the graph
        """

        distances = []

        for pt in self.graph.vertices:
            
            distances.append(np.linalg.norm(np.array(pt) - np.array(node)))

        return np.argmin(distances)
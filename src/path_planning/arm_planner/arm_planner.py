"""
Planning algorithms for arm 
"""

import cv2
import numpy as np
import heapq

class ArmPlanner:
    
    def __init__(self, current_j1, current_j2, desired_j1, desired_j2, costmap, robot_arm, do_viz=False, max_iters=np.inf):
        
        self.current_j1 = current_j1
        self.current_j2 = current_j2

        self.desired_j1 = desired_j1
        self.desired_j2 = desired_j2

        self.costmap = costmap

        self.do_viz = do_viz

        self.increment = 1
        self.goal_thresh = 3e-2

        self.robot_arm = robot_arm

        self.joint_1_lim = [-np.pi/2, np.pi/2]
        self.joint_2_lim = [-np.pi/2, np.pi/2]

        self.max_iters = max_iters

        self.grid_size = 200
        self.step_size = 2 * (2*np.pi / self.grid_size)

        #self.configuration_space = np.zeros((self.grid_size, self.grid_size))

    def continous_to_discrete_joint_space(self, joint_1, joint_2):

        joint_1_discrete = np.ceil(joint_1 / self.step_size) + self.grid_size // 2
        joint_2_discrete = np.ceil(joint_2 / self.step_size) + self.grid_size // 2

        return joint_1_discrete, joint_2_discrete

    def discrete_to_continuous_joint_space(self, grid_x, grid_y):

        joint_1_continuous = (grid_x - self.grid_size // 2) * self.step_size
        joint_2_continuous = (grid_y - self.grid_size // 2) * self.step_size

        return joint_1_continuous, joint_2_continuous

    def check_collision(self, joint_1, joint_2):
        """
        Check for collision with given arm configuration
        """

        self.robot_arm.set_joint_angles(joint_1, joint_2, convert_to_radians=False)
        points_link_1, points_link_2 = self.robot_arm.calc_transformed_points()
        self.robot_arm.set_joint_angles(self.current_j1, self.current_j2, convert_to_radians=False)

        viz_im = self.costmap.copy()

        for point in points_link_1:
            point_world = self.robot_arm.convert_to_world_frame(point[0], point[1])
            
            if (self.costmap[point_world[1], point_world[0]] == (0, 0, 0)).all():
                return True

            #cv2.circle(viz_im, point_world, 6, (0, 0,20), -1)

            #obstacle_points = np.where(self.costmap == (0, 0, 0))[:2]
            #query = (obstacle_points[0][0], obstacle_points[1][0])
            #print ()
            #print (query)
            #print (self.costmap[query[0], query[1]])
            #print ()
            #print (point_world, self.costmap[point_world[0], point_world[1]])

            #cv2.imshow("debug", viz_im)

        for point in points_link_2:
            point_world = self.robot_arm.convert_to_world_frame(point[0], point[1])

            if (self.costmap[point_world[1], point_world[0]] == (0, 0, 0)).all():
                return True

            #cv2.circle(viz_im, point_world, 6, (20, 0,20), -1)

        #cv2.imshow("debug", viz_im)
        #cv2.waitKey(1)

        return False

    def bfs(self):

        print ("Pathing toward: ")
        print (self.desired_j1)
        print (self.desired_j2)

        start_point = [self.current_j1, self.current_j2]
        queue = [[start_point]]
        visited = set()

        while queue:

            path = queue.pop(0)
            node = path[-1]

            #print (node)

            if tuple(node) in visited:
                continue

            error = ((node[0] - self.desired_j1)**2 + (node[1] - self.desired_j2)**2)**0.5

            if error <= self.goal_thresh:
                return path

            neigh_left = [node[0] - self.increment, node[1]]
            neigh_right = [node[0] + self.increment, node[1]]
            neigh_up = [node[0], node[1] - self.increment]
            neigh_down = [node[0], node[1] + self.increment]

            path_ = path[:]
            path_.append(neigh_left)

            if not self.check_collision(neigh_left[0], neigh_left[1]):
                queue.append(path_)

            path_ = path[:]
            path_.append(neigh_right)

            if not self.check_collision(neigh_right[0], neigh_right[1]):
                queue.append(path_)

            path_ = path[:]
            path_.append(neigh_up)
            
            if not self.check_collision(neigh_up[0], neigh_up[1]):
                queue.append(path_)

            path_ = path[:]
            path_.append(neigh_down)
            
            if not self.check_collision(neigh_down[0], neigh_down[1]):
                queue.append(path_)

            visited.add(tuple(node))


    def dist(self, pair_1):

        return ((pair_1[0] - self.desired_j1)**2 + (pair_1[1] - self.desired_j2)**2)**0.5

    def dist_continous(self, pair_1, pair_2):

        return ((pair_1[0] - pair_2[0])**2 + (pair_1[1] - pair_2[1])**2)**0.5
    
    def a_star(self):

        print ("Checking feasibility of goal point...")
        print ()
        if self.check_collision(self.desired_j1, self.desired_j2):
            return None

        print ("Goal point is feasible!")
        print()
        print ("Pathing toward: ")
        print (self.desired_j1)
        print (self.desired_j2)
        print ()

        start_point = self.continous_to_discrete_joint_space(self.current_j1, self.current_j2)
        goal_point = self.continous_to_discrete_joint_space(self.desired_j1, self.desired_j2)

        print (goal_point)
        print ()
        
        start_point = (start_point[0], start_point[1])
        queue = []
        heapq.heappush(queue, (0, [start_point]))
        visited = set()
        iter_count = 0

        while queue:

            node_cost, path = heapq.heappop(queue)
            node = path[-1]
            #print (node_cost, node)

            #print ("debug")
            #print (node)

            if node in visited:
                continue

            #print (node_hash)

            if node == goal_point:

                path_continuous = []
                for path_point in path:
                    path_continuous.append(self.discrete_to_continuous_joint_space(path_point[0], path_point[1]))

                return path_continuous

            potential_neigh = []
            
            potential_neigh.append((node[0] - self.increment, node[1]))
            potential_neigh.append((node[0] + self.increment, node[1]))
            potential_neigh.append((node[0], node[1] - self.increment))
            potential_neigh.append((node[0], node[1] + self.increment))
            potential_neigh.append((node[0] + self.increment, node[1] + self.increment))
            potential_neigh.append((node[0] - self.increment, node[1] - self.increment))
            potential_neigh.append((node[0] + self.increment, node[1] - self.increment))
            potential_neigh.append((node[0] - self.increment, node[1] + self.increment))

            
            for neigh in potential_neigh:

                path_ = path.copy()

                if neigh[0] < 0 or neigh[0] > self.grid_size or neigh[1] < 0 or neigh[1] > self.grid_size:
                    continue

                path_.append(neigh)

                """
                if not (neigh[0] >= self.joint_1_lim[0] and neigh[0] <= self.joint_1_lim[1]):
                    continue

                if not (neigh[1] >= self.joint_2_lim[0] and neigh[1] <= self.joint_2_lim[1]):
                    continue
                """

                joint_1_continuous, joint_2_continuous = self.discrete_to_continuous_joint_space(neigh[0], neigh[1])

                if not self.check_collision(joint_1_continuous, joint_2_continuous):
                    heapq.heappush(queue, (self.dist_continous(neigh, goal_point), path_))

            visited.add(node)
            iter_count += 1

            if iter_count == self.max_iters:
                return None
from arm import RobotArm
from arm_planner import ArmPlanner
import numpy as np
import cv2

if __name__ == '__main__':

    

    arm = RobotArm(link_1_length=60, link_2_length=90, joint_1=0, joint_2=0)
    
    world_size = 700
    world = np.ones((world_size, world_size, 3)) * 255

    cv2.rectangle(world, (280, 300), (375, 335), (0, 0, 0), -1)
    cv2.rectangle(world, (445, 290), (535, 315), (0, 0, 0), -1)
    cv2.rectangle(world, (415, 320), (535, 340), (0, 0, 0), -1)

    desired_x_world = 50 + 350
    desired_y_world = -98 + 350

    cv2.circle(world, (desired_x_world, desired_y_world), 5, (0, 0, 200), -1)

    #world[desired_y_world, desired_x_world] = [0, 0, 0]

    desired_x_arm, desired_y_arm = arm.convert_to_arm_frame(desired_x_world, desired_y_world)

    #print (desired_x_arm, desired_y_arm)
    
    desired = arm.backward(desired_x_arm, desired_y_arm)

    planner = ArmPlanner(0, 0, desired[0], desired[1], world, arm)

    path = planner.a_star()

    viz_frame = world.copy()

    for pt in arm.link_1_origin_points:
        cv2.circle(viz_frame, (pt[0] + world_size // 2, pt[1] + world_size // 2), 3, (200, 0, 0), 1)

    for pt in arm.link_2_origin_points:
        cv2.circle(viz_frame, (pt[0] + world_size // 2, pt[1] + world_size // 2), 3, (0, 200, 0), 1)

    cv2.imshow("world", viz_frame)
    cv2.waitKey(0)

    if path is None:

        print ("Could not find a path with solution 1... Inverting joint_2")

        desired = arm.backward(desired_x_arm, desired_y_arm, joint_2_mult=-1)
        planner = ArmPlanner(0, 0, desired[0], desired[1], world, arm)

        path = planner.a_star()

        if path is None:
            print ("no solution found")
            exit()


    for sub_pt in path:
    
        arm.set_joint_angles(sub_pt[0], sub_pt[1], convert_to_radians=False)
        arm.calc_transformed_points()

        viz_frame = world.copy()

        for pt in arm.link_1_origin_points:
            cv2.circle(viz_frame, (pt[0] + world_size // 2, pt[1] + world_size // 2), 3, (200, 0, 0), 1)

        for pt in arm.link_2_origin_points:
            cv2.circle(viz_frame, (pt[0] + world_size // 2, pt[1] + world_size // 2), 3, (0, 200, 0), 1)

        ee_pos_arm_frame = arm.forward()
        ee_pos_world_frame = arm.convert_to_world_frame(ee_pos_arm_frame[0], ee_pos_arm_frame[1])

        
        cv2.circle(viz_frame, (ee_pos_world_frame[0], ee_pos_world_frame[1]), 5, (0, 0, 0), -1)
        cv2.circle(viz_frame, (desired_x_world, desired_y_world), 5, (0, 0, 200), -1)

        #print (ee_pos_world_frame)

        cv2.imshow("world", viz_frame)
        cv2.waitKey(100)

    cv2.waitKey(0)
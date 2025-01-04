#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import py_trees

from utils.lidar import LidarProcessor

from behaviours.decide_movement import DecideMovement
from behaviours.find_doors import FindDoors
from behaviours.fixed_movement import FixedMovement
from behaviours.move_to_door import MoveToDoor
from behaviours.obstacle_detection import ObstacleDetection
from behaviours.random_rotation import RandomSafeRotation
from behaviours.stuck_detection import StuckDetection

def main():
    rospy.init_node('behaviour_tree_node')

    robot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    lidar_processor = LidarProcessor()
    rospy.Subscriber('/scan', LaserScan, lidar_processor.lidar_callback)
    
    # Behavior Tree Structure
    root = py_trees.composites.Selector("Root", memory=True, children=[
        py_trees.composites.Sequence(
            "MoveForward", 
            memory=False, # Important no memory to force to execute repeatedly.
            children=[
                ObstacleDetection(robot_vel_pub, fixed_direction_angle=0.0), 
                FixedMovement(robot_vel_pub, linear_speed=0.2)
            ]
        ),
        py_trees.composites.Sequence(
            "RetryDirection", 
            memory=False, # Important no memory to force to execute repeatedly.
            children=[
                # How to optionally move backwards and rotate?
                py_trees.composites.Sequence(
                    "MoveBackwards",
                    memory=False,
                    children = [
                        ObstacleDetection(robot_vel_pub, fixed_direction_angle=180.0),
                        FixedMovement(robot_vel_pub, linear_speed=-0.2, max_time=1.0),
                    ]
                ),
                RandomSafeRotation(robot_vel_pub)
            ]
        ),
    ])

    tree = py_trees.trees.BehaviourTree(root)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()


if __name__ == '__main__':
    main()
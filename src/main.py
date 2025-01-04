#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import py_trees

from utils.lidar import LidarProcessor

from behaviours.decide_movement import DecideMovement
from behaviours.find_doors import FindDoors
from behaviours.move_backward import MoveBackward
from behaviours.move_forward import MoveForward
from behaviours.move_to_door import MoveToDoor
from behaviours.obstacle_detection import ObstacleDetection
from behaviours.random_rotation import RandomRotation
from behaviours.stuck_detection import StuckDetection

def main():
    rospy.init_node('behaviour_tree_node')
    robot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    lidar_processor = LidarProcessor()
    rospy.Subscriber('/scan', LaserScan, lidar_processor.lidar_callback)
    
    # Create the blackboard
    blackboard = py_trees.blackboard.Blackboard()

    # Behaviors
    move_forward = MoveForward(robot_vel_pub)
    move_backward = MoveBackward(robot_vel_pub)
    obstacle_detection = ObstacleDetection(lidar_processor, robot_vel_pub)
    find_doors = FindDoors(lidar_processor)
    move_to_door = MoveToDoor(lidar_processor, robot_vel_pub)
    random_rotation = RandomRotation(robot_vel_pub)
    stuck_detection = StuckDetection(lidar_processor, robot_vel_pub, move_forward)
    decide_movement=DecideMovement(lidar_processor, robot_vel_pub)
    
    # Behavior Tree Structure
    root = py_trees.composites.Sequence("Root", memory=blackboard, children=[move_forward])

    tree = py_trees.trees.BehaviourTree(root)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()


if __name__ == '__main__':
    main()
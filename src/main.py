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
        # Repeatedly move forward as long as there is no obstacle.
        py_trees.composites.Sequence(
            "MoveForward", 
            memory=False,
            children=[
                ObstacleDetection(robot_vel_pub, fixed_direction_angle=0.0), 
                FixedMovement(robot_vel_pub, linear_speed=0.2)
            ]
        ),
        # Otherwise, try to move backwards and rotate randomly.
        py_trees.composites.Sequence(
            "RetryDirection", 
            memory=True,
            children=[
                # Optionally try to move backwards. If not possible, continue.
                py_trees.decorators.FailureIsSuccess(
                    "TryBackoff",
                    py_trees.composites.Sequence(
                        "Backoff",
                        memory=False,
                        children = [
                            # Try to move backwards a fixed ammount of time.
                            ObstacleDetection(robot_vel_pub, fixed_direction_angle=180.0),
                            py_trees.decorators.Timeout(
                                "MoveBackwards",
                                duration=1.0,
                                child=FixedMovement(robot_vel_pub, linear_speed=-0.2),
                            )
                        ]
                    )
                ),
                # Temperature specifies how biased is the robot towards open spaces.
                # The lower, the more likely it is the direction is a big open space.
                # Note that low values could result in the robot moving back and forth
                # in big closed spaces.
                RandomSafeRotation(robot_vel_pub, temperature=0.1)
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
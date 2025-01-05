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

    lidar_processor = LidarProcessor()
    rospy.Subscriber('/scan', LaserScan, lidar_processor.lidar_callback)
    
    # Behavior Tree Structure
    root = py_trees.composites.Selector("Root", memory=True, children=[
        # Repeatedly move forward as long as there is no obstacle.
        py_trees.composites.Sequence(
            "MoveForward", 
            memory=False,
            children=[
                ObstacleDetection(fixed_direction_angle=0.0, obstacle_threshold=1.5), 
                FixedMovement(linear_speed=0.2)
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
                            # Try to move backwards as long as there are obstacles 1 meter around the robot.
                            py_trees.decorators.Inverter(
                                "ObstacleInFront",
                                child = ObstacleDetection( 
                                    fixed_direction_angle=None,
                                    obstacle_threshold=1.0,
                                    adjust_based_on_speed=False
                                ),
                            ),
                            # Only move backwards if there is no obstacle behind us.
                            ObstacleDetection(fixed_direction_angle=180.0, obstacle_threshold=1.5),
                            # Try to move backwards 2 seconds or give up.
                            py_trees.decorators.Timeout(
                                "MoveBackwards",
                                duration=2.0,
                                child=FixedMovement(linear_speed=-0.2),
                            )
                        ]
                    )
                ),
                # Temperature specifies how biased is the robot towards open spaces.
                # The lower, the more likely it is the direction is a big open space.
                # Note that low values could result in the robot moving back and forth
                # in big closed spaces.
                RandomSafeRotation(temperature=0.1)
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
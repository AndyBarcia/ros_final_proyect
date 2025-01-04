import py_trees
from geometry_msgs.msg import Twist
import numpy as np
import math

class ObstacleDetection(py_trees.behaviour.Behaviour):
    def __init__(
        self, 
        robot_vel_pub, 
        fixed_direction_angle=0.0,
        name="ObstacleDetection", 
        obstacle_threshold=0.5
    ):
        super(ObstacleDetection, self).__init__(name)

        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key="safe_distances", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="sensor_angles", access=py_trees.common.Access.READ)

        self.fixed_direction_angle = fixed_direction_angle
        self.robot_vel_pub = robot_vel_pub
        self.obstacle_threshold = obstacle_threshold
        self.robot_vel_pub = robot_vel_pub

    def get_max_distance_in_direction(self,grad_angle):
        if self.blackboard.exists("safe_distances") and self.blackboard.exists("sensor_angles"):
            # Find the closest angle index in sensor_angles
            angle = grad_angle * math.pi / 180.0
            closest_index = np.argmin(np.abs(np.array(self.blackboard.sensor_angles) - angle))
            return self.blackboard.safe_distances[closest_index]
        else:
            # Otherwise, assume the safe distance is 0.
            return 0

    def update(self):
        # Get safe distance along given forward direction.
        forward_safe_distance = self.get_max_distance_in_direction(self.fixed_direction_angle)

        if forward_safe_distance < self.obstacle_threshold:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.robot_vel_pub.publish(twist)

            # Fail so that no other behaviours in the tree get executed.
            return py_trees.common.Status.FAILURE
        else:

            # If no obstacle, we can continue.
            return py_trees.common.Status.SUCCESS
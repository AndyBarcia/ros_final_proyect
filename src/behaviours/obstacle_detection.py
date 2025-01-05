import py_trees
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import numpy as np
import math

class ObstacleDetection(py_trees.behaviour.Behaviour):
    def __init__(
        self, 
        fixed_direction_angle=0.0,
        name="ObstacleDetection", 
        obstacle_threshold=0.15,
        adjust_based_on_speed=True
    ):
        super(ObstacleDetection, self).__init__(name)

        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key="safe_distances", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="sensor_angles", access=py_trees.common.Access.READ)

        self.fixed_direction_angle = fixed_direction_angle
        
        # The reference distance to consider an obstacle when the robot is going 1m/s.
        # When going slower, the distance becomes smaller as it is easier to stop.
        self.obstacle_threshold = obstacle_threshold
        self.adjust_based_on_speed = adjust_based_on_speed

        self.robot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odometry_callback)

        self.current_speed = 0.0

    def odometry_callback(self, msg):
        # Update the current speed from the linear.x component of the Twist message
        linear_velocity = msg.twist.twist.linear
        self.current_speed = math.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)

    def get_max_distance_in_direction(self, grad_angle):
        if self.blackboard.exists("safe_distances") and self.blackboard.exists("sensor_angles"):
            # Find the closest angle index in sensor_angles
            if grad_angle:
                angle = grad_angle * math.pi / 180.0
                closest_index = np.argmin(np.abs(np.array(self.blackboard.sensor_angles) - angle))
                return self.blackboard.safe_distances[closest_index]
            else:
                return np.array(self.blackboard.safe_distances).min()
        else:
            # Otherwise, report that we don't know the safe distance.
            return None

    def update(self):
        # Get safe distance along given forward direction.
        forward_safe_distance = self.get_max_distance_in_direction(self.fixed_direction_angle)
        
        # If we still don't know the safe distance because LIDAR didn't boot up, keep running.
        if not forward_safe_distance:
            return py_trees.common.Status.RUNNING

        # Given that to stop an object of mass m at speed v along a distance d the following holds:
        #  F*d = 1/2*m*v^2
        # And for the robot the force F and mass m remains constant, then we have that
        #  d ≈ v^2
        # So the stopping distance is proportional to velocity square.
        # Knowing this, we can simply scale the reference stopping distance by velocity square.
        threshold = self.obstacle_threshold*self.current_speed*self.current_speed if self.adjust_based_on_speed else self.obstacle_threshold
        if forward_safe_distance < threshold:
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
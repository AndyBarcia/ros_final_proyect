import rospy
import py_trees
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import random
import numpy as np

class RandomSafeRotation(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        robot_vel_pub,
        angular_speed=0.3,
        name="RandomRotation",
        obstacle_threshold=0.5,
        angle_threshold=0.1  # Threshold for considering target reached (radians)
    ):
        super(RandomSafeRotation, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key="safe_distances", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="sensor_angles", access=py_trees.common.Access.READ)
        
        self.robot_vel_pub = robot_vel_pub
        self.angular_speed = angular_speed
        self.angle_threshold = angle_threshold
        self.obstacle_threshold = obstacle_threshold
        
        # Subscribe to odometry to get robot's orientation
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0.0
        self.target_safe_angle = None
        
    def generate_random_safe_distance(self):
        safe_distances = np.array(self.blackboard.safe_distances)
        safe_angles = np.array(self.blackboard.sensor_angles)
        safe_mask = safe_distances > self.obstacle_threshold
        
        if len(safe_angles) == 0:
            self.target_safe_angle = None
            return False

        # Random safe angle
        self.target_safe_angle = np.random.choice(safe_angles[safe_mask]) + self.current_yaw

    def odom_callback(self, msg):
        # Get orientation from quaternion
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        
    def update(self):
        if self.target_safe_angle is None:
            if not self.generate_random_safe_distance():
                print("NO SAFE ANGLES")
                return py_trees.common.Status.FAILURE
            print("NO SAFE ANGLES")
        
        # Calculate relative angle to target
        relative_angle = self.target_safe_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi]
        while relative_angle > np.pi:
            relative_angle -= 2 * np.pi
        while relative_angle < -np.pi:
            relative_angle += 2 * np.pi
        
        # Check if target rotation is reached
        reached_target_rotation = abs(relative_angle) < self.angle_threshold
        
        if not reached_target_rotation:
            twist = Twist()
            # Adjust rotation direction based on shortest path
            if relative_angle > 0:
                twist.angular.z = self.angular_speed
            else:
                twist.angular.z = -self.angular_speed
            self.robot_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Stop rotation
            twist = Twist()
            twist.angular.z = 0.0
            self.robot_vel_pub.publish(twist)

            # Reset targets
            self.target_safe_angle = None

            return py_trees.common.Status.SUCCESS
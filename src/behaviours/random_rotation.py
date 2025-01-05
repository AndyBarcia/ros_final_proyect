import rospy
import py_trees
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math

class RandomSafeRotation(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        robot_vel_pub,
        angular_speed=0.3,
        name="RandomRotation",
        obstacle_threshold=0.01,
        angle_threshold=0.1,  # Threshold for considering target reached (radians)
        temperature=1.0       # Temperature parameter for softmax
    ):
        super(RandomSafeRotation, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key="safe_distances", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="sensor_angles", access=py_trees.common.Access.READ)
        self.robot_vel_pub = robot_vel_pub
        self.angular_speed = angular_speed
        self.angle_threshold = angle_threshold
        self.obstacle_threshold = obstacle_threshold
        self.temperature = temperature
        
        # Subscribe to odometry to get robot's orientation
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0.0
        self.target_safe_angle = None

    def generate_random_safe_direction(self):
        safe_distances = np.array(self.blackboard.safe_distances)
        safe_angles = np.array(self.blackboard.sensor_angles)
        
        # Filter out unsafe distances
        safe_mask = safe_distances > self.obstacle_threshold
        safe_distances[~safe_mask] = -math.inf

        # If no safe distance, abort.
        if sum(safe_mask) == 0:
            return None
        
        # Clip maxium distance so that they aren't always picked.
        # safe_distances = np.clip(safe_distances, a_min=None, a_max=self.max_distance)
                
        # Apply softmax with temperature
        exp_distances = np.exp(safe_distances / self.temperature)
        probabilities = exp_distances / np.sum(exp_distances)
        
        # Choose angle based on calculated probabilities
        return np.random.choice(safe_angles, p=probabilities) + self.current_yaw

    def initialise(self):
        # Try to generate a random safe direction at start.
        self.target_safe_angle = self.generate_random_safe_direction()

    def terminate(self, new_status: py_trees.common.Status):
        # Stop rotation
        twist = Twist()
        twist.angular.z = 0.0
        self.robot_vel_pub.publish(twist)
        
        # Reset target angle
        self.target_safe_angle = None

    def odom_callback(self, msg):
        # Get orientation from quaternion
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def update(self):
        if self.target_safe_angle is None:
            self.target_safe_angle = self.generate_random_safe_direction()
            if not self.target_safe_angle:
                print("NO SAFE ANGLES")
                return py_trees.common.Status.FAILURE
        
        # Calculate relative angle to target
        relative_angle = self.target_safe_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi]
        while relative_angle > np.pi:
            relative_angle -= 2 * np.pi
        while relative_angle < -np.pi:
            relative_angle += 2 * np.pi
            
        print(f"ROTATING TOWARDS {relative_angle}")
        
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
            return py_trees.common.Status.SUCCESS
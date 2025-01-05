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
        self.angular_speed = angular_speed
        self.angle_threshold = angle_threshold
        self.obstacle_threshold = obstacle_threshold
        self.temperature = temperature
        
        # Subscribe to odometry to get robot's orientation
        self.robot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
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
        
        # Create position weights that favor the front
        num_angles = len(safe_angles)
        mid_point = num_angles // 2
        
        # Create weights that decrease as we move away from the front (0 degrees)
        # Use cosine function to create smooth weighting
        position_weights = np.cos(np.abs(safe_angles) * np.pi / 180)
        
        # Normalize position weights to be between 0.5 and 1 to maintain some probability
        # for all directions while still favoring the front
        position_weights = 0.5 + 0.5 * position_weights
        
        # Combine distance information with position weights
        weighted_distances = safe_distances * position_weights
        
        # Apply softmax with temperature
        exp_distances = np.exp(weighted_distances / self.temperature)
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
        # Normalize angle to [-pi, pi]
        relative_angle = self.target_safe_angle
        while relative_angle > np.pi:
            relative_angle -= 2 * np.pi
        while relative_angle < -np.pi:
            relative_angle += 2 * np.pi
        
        relative_angle = relative_angle - self.current_yaw

        print(f"ROTATING TOWARDS {relative_angle}")
        
        # Check if target rotation is reached
        reached_target_rotation = abs(relative_angle) < self.angle_threshold
        
        if not reached_target_rotation:
            twist = Twist()
            # Adjust rotation direction based on shortest path. Instead of directly setting
            # the speed, used sigmoid function to rapidly decrease the rotation speed when 
            # we are near the target rotation. This avoid jerky movement.
            twist.angular.z = self.angular_speed * 2 / (1 + math.exp(-50*relative_angle)) - 1
            self.robot_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
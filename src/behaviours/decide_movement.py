import rospy
import py_trees
import numpy as np
from geometry_msgs.msg import Twist
import math

FOV_FRONT = math.pi / 2  # Front field of view (90 degrees)
SAFE_DISTANCE_FRONT = 0.8  # Distance for moving forward
SAFE_DISTANCE_SIDES = 0.6  # Distance for selecting a side direction
FORWARD_SPEED = 0.2
REVERSE_SPEED = -0.1
ROTATE_SPEED = 0.3

def get_closes_safest_direction(max_safe_distances, angles, safe_distance_sides):
    """Get the index of the closest safest direction."""
    safe_directions = np.where(max_safe_distances > safe_distance_sides)[0]
    if safe_directions.size == 0:
        return None

    # Find the angle that is closest to 0 (forward direction)
    closest_angle = float('inf')
    closest_index = None
    for i in safe_directions:
         angle_diff= abs(angles[i])
         if angle_diff < closest_angle:
             closest_angle=angle_diff
             closest_index = i
    return closest_index

class DecideMovement(py_trees.behaviour.Behaviour):
    def __init__(self, lidar_processor, robot_vel_pub, name="DecideMovement"):
        super(DecideMovement, self).__init__(name)
        self.lidar_processor = lidar_processor
        self.robot_vel_pub = robot_vel_pub
        self.velocity_msg = Twist()

    def decide_movement(self, max_safe_distances, angles):
        """Decide robot movement based on maximum safe distances."""
        front_mask = (angles > -FOV_FRONT / 2) & (angles < FOV_FRONT / 2)
        front_distances = max_safe_distances[front_mask]

        # If the front is clear, move forward
        if np.all(front_distances > SAFE_DISTANCE_FRONT):
           self.velocity_msg.linear.x = FORWARD_SPEED
           self.velocity_msg.angular.z = 0.0
           rospy.loginfo("Moving forward")
        else:
            # Find the closest safe direction
            safe_direction_index = get_closes_safest_direction(
                max_safe_distances, 
                angles, 
                SAFE_DISTANCE_SIDES
            )

            # If no safe direction, reverse
            if safe_direction_index is None:
                self.velocity_msg.linear.x = REVERSE_SPEED
                self.velocity_msg.angular.z = 0.0
                rospy.loginfo("No safe direction, reversing")
            else:
                # Determine turn direction and speed
                best_direction_angle = angles[safe_direction_index]
                
                # Turn towards the closest safe direction
                turn_speed = ROTATE_SPEED * np.sign(best_direction_angle)
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = turn_speed
                
                turn_direction = 'left' if turn_speed > 0 else 'right'
                rospy.loginfo(f"Turning {turn_direction} to closest safe direction")

    def update(self):
        safe_distances = self.lidar_processor.get_safe_distances()
        sensor_angles = self.lidar_processor.sensor_angles
        if len(safe_distances) == 0 or sensor_angles is None:
             return py_trees.common.Status.FAILURE
        
        self.decide_movement(safe_distances,sensor_angles)
        self.robot_vel_pub.publish(self.velocity_msg)
        return py_trees.common.Status.RUNNING
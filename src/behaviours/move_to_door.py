import py_trees
from geometry_msgs.msg import Twist

class MoveToDoor(py_trees.behaviour.Behaviour):
    def __init__(self, lidar_processor, robot_vel_pub, name="MoveToDoor", door_distance_threshold=0.2):
        super(MoveToDoor, self).__init__(name)
        self.lidar_processor = lidar_processor
        self.robot_vel_pub = robot_vel_pub
        self.door_locations = []
        self.linear_speed = 0.1
        self.angular_speed = 0.1
        self.door_distance_threshold = door_distance_threshold

    def update(self):
        if not self.door_locations:
            self.door_locations = self.lidar_processor.get_door_locations()

        if not self.door_locations:
            return py_trees.common.Status.FAILURE  # no doors to move

        # We will go for the first door
        door_angle = self.door_locations[0]
        max_dist_door = self.lidar_processor.get_max_distance_in_direction(door_angle)

        if max_dist_door < self.door_distance_threshold:
            return py_trees.common.Status.SUCCESS  # we have reached the door, no need to keep going

        twist = Twist()
        # Calculate the error of the current angle and the angle of the door
        angle_error = door_angle
        twist.angular.z = self.angular_speed * angle_error  # turn until facing the door
        twist.linear.x = self.linear_speed

        if abs(angle_error) < 0.1:  # if it is facing the door, move forward
            twist.angular.z = 0  # no turning

        self.robot_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING
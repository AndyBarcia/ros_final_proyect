import py_trees
from geometry_msgs.msg import Twist

class ObstacleDetection(py_trees.behaviour.Behaviour):
    def __init__(self, lidar_processor, robot_vel_pub, name="ObstacleDetection", obstacle_threshold=0.5):
        super(ObstacleDetection, self).__init__(name)
        self.lidar_processor = lidar_processor
        self.obstacle_threshold = obstacle_threshold
        self.robot_vel_pub = robot_vel_pub

    def update(self):

        safe_distances = self.lidar_processor.get_safe_distances()
        if len(safe_distances) == 0:
            return py_trees.common.Status.FAILURE

        # Forward direction is assumed to be angle 0.
        forward_safe_distance = self.lidar_processor.get_max_distance_in_direction(0.0)

        if forward_safe_distance < self.obstacle_threshold:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.robot_vel_pub.publish(twist)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
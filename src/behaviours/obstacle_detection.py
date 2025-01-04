import py_trees
from geometry_msgs.msg import Twist

class ObstacleDetection(py_trees.behaviour.Behaviour):
    def __init__(self, lidar_processor, robot_vel_pub):
        super(ObstacleDetection, self).__init__(name="ObstacleDetection")
        self.lidar_processor = lidar_processor
        self.robot_vel_pub = robot_vel_pub

    def update(self):
        if self.lidar_processor.detect_obstacles(robot_radius=0.3):  # Example robot radius
            twist = Twist()
            twist.angular.z = 0.5  # Rotate to avoid obstacle
            self.robot_vel_pub.publish(twist)
            print("Publishing twist message to rotate")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
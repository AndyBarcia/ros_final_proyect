import rospy
import py_trees
from geometry_msgs.msg import Twist
import random

class RandomRotation(py_trees.behaviour.Behaviour):
    def __init__(self, robot_vel_pub, name="RandomRotation", rotation_duration=2):
        super(RandomRotation, self).__init__(name)
        self.robot_vel_pub = robot_vel_pub
        self.rotation_duration = rotation_duration
        self.start_time = None
        self.angular_speed = 0.3  # Angular speed for rotation
        self.rotation_direction = 1  # 1 for clockwise, -1 for counterclockwise

    def update(self):
        if self.start_time is None:
            self.start_time = rospy.Time.now()
            self.rotation_direction = random.choice([1,-1])  # Randomize rotation direction
        
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()

        if elapsed_time < self.rotation_duration:
            twist = Twist()
            twist.angular.z = self.angular_speed * self.rotation_direction
            self.robot_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            self.start_time = None
            return py_trees.common.Status.SUCCESS
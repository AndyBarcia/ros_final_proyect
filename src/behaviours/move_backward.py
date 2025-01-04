import py_trees
from geometry_msgs.msg import Twist

class MoveBackward(py_trees.behaviour.Behaviour):
    def __init__(self, robot_vel_pub, name="MoveBackward"):
        super(MoveBackward, self).__init__(name)
        self.robot_vel_pub = robot_vel_pub
        self.linear_speed = -0.2  # meters per second
        self.angular_speed = 0
        
    def update(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.robot_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING
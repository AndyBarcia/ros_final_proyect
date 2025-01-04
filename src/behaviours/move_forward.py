import py_trees
from geometry_msgs.msg import Twist

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, robot_vel_pub, name="MoveForward"):
        super(MoveForward, self).__init__(name)
        self.robot_vel_pub = robot_vel_pub
        self.linear_speed = 0.2  # meters per second
        self.angular_speed = 0
        self.moving_forward=True

    def update(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.robot_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def get_moving_forward(self):
        return self.moving_forward
    
    def set_moving_forward(self, moving_forward):
        self.moving_forward=moving_forward
import py_trees
from geometry_msgs.msg import Twist

class FixedMovement(py_trees.behaviour.Behaviour):
    def __init__(
        self, 
        robot_vel_pub, 
        linear_speed = 0.2,  # meters per second
        angular_speed = 0,
        name="FixedMovement",
    ):
        super(FixedMovement, self).__init__(name)
        self.robot_vel_pub = robot_vel_pub
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.moving_forward=True

    def terminate(self, new_status: py_trees.common.Status):
        # Stop the robot when behaviour terminated.
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.robot_vel_pub.publish(twist)

    def update(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.robot_vel_pub.publish(twist)

        print(f"MOVING {self.linear_speed}")

        return py_trees.common.Status.RUNNING

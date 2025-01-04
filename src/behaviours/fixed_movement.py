import py_trees
from geometry_msgs.msg import Twist
import time

class FixedMovement(py_trees.behaviour.Behaviour):
    def __init__(
        self, 
        robot_vel_pub, 
        linear_speed = 0.2,  # meters per second
        angular_speed = 0,
        name="FixedMovement",
        max_time=None
    ):
        super(FixedMovement, self).__init__(name)
        self.robot_vel_pub = robot_vel_pub
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.moving_forward=True
        self.max_time = max_time
        self.start_time = None

    def update(self):
        if self.start_time is None and self.max_time is not None:
            self.start_time = time.time()
    
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.robot_vel_pub.publish(twist)

        if self.max_time is None or time.time() - self.start_time < self.max_time:
            print("MOVING")
            # Still within the time limit, keep running the behavior
            return py_trees.common.Status.RUNNING
        else:
            # Time is up, stop the behavior
            return py_trees.common.Status.SUCCESS

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from diffdrive_msgs.msg import Pose2DStamped
from geometry_msgs.msg import PointStamped
from webots_ros2_msgs.msg import FloatStamped

import math


class RobotStatePublisher(Node):
    """
    Robot state publisher node, collects information from 
    various sensors and packages them to publish robot state.
    """
    def __init__(self):
        super().__init__("robot_state_publisher")

        self.declare_parameter("robot_name", value="")
        self.declare_parameter("frequency", value=30)

        # Since the robot is considered as a differential drive robot 
        # state is defined as position and orientation of the robot. 
        # Position info is measured through gps and the orientation is 
        # measured through compass
        self.theta = 0.
        self.px = 0.
        self.py = 0.
        # self.last_pub_time = Time()

        timer_period = 1 / self.get_parameter("frequency").value
        self.create_timer(timer_period, self.timer_callback)

        self.create_subscription(
            FloatStamped, 
            self.get_parameter('robot_name').value
                + "/compass/bearing", 
            self.compass_callback, 
            qos_profile=1
            )
        self.create_subscription(
            PointStamped,
            self.get_parameter('robot_name').value
                + "/gps",
            self.gps_callback,
            qos_profile=1
            )
        
        self._state_publisher = self.create_publisher(
            Pose2DStamped, 
            self.get_parameter('robot_name').value + '/state', 
            qos_profile=1
            )
        
        self.clock = self.get_clock()

    def gps_callback(self, gps_msg:PointStamped) -> None:
        self.px = gps_msg.point.x
        self.py = gps_msg.point.y
        return
    
    def compass_callback(self, compass_msg:FloatStamped) -> None:
        dummy =  360.0 - compass_msg.data
        self.theta = math.radians(dummy)
        return

    def timer_callback(self) -> None:
        """
        Publish state at each timer step.
        """
        state = Pose2DStamped()
        state.pose.x = self.px
        state.pose.y = self.py
        state.pose.theta = self.theta
        state.timestamp = self.clock.now().to_msg()
        self._state_publisher.publish(state)
        return 

def main(args=None) -> None:
    rclpy.init(args=args)

    state_publisher = RobotStatePublisher()

    try:
        rclpy.spin(state_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        state_publisher.destroy_node()
        rclpy.try_shutdown()
    return

if __name__ == "__main__":
    main()
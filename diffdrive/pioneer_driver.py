import rclpy

from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

WHEEL_RADIUS = 0.111 # in meters
HALF_WHEEL_BASE = 0.197 # in meters
MAX_WHEEL_SPEED = 6.4 # radians per second

class PioneerDriver():
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.robot_name = self.robot.getName()

        self.des_lin_speed = 0.0
        self.des_ang_speed = 0.0

        self.left_motors = []
        self.right_motors = []
        # Get Controllable motors
        self.left_motors.append(self.robot.getDevice("front left wheel"))
        self.left_motors.append(self.robot.getDevice("back left wheel"))
        self.right_motors.append(self.robot.getDevice("front right wheel"))
        self.right_motors.append(self.robot.getDevice("back right wheel"))
        
        # Assign motor speeds to 0 and position to infinity
        for motor in self.left_motors:
            motor.setVelocity(0)
            motor.setPosition(float('inf'))
        for motor in self.right_motors:
            motor.setVelocity(0)
            motor.setPosition(float('inf'))
        
        rclpy.init(args=None)
        self._node = rclpy.create_node('pioneer_driver')
        self._node.create_subscription(Twist, self.robot_name+'/cmd_vel', 
            self.des_speed_callback, 1)

        pass

    def des_speed_callback(self, msg:Twist):
        self.des_lin_speed = msg.linear.x
        self.des_ang_speed = msg.angular.z
    
    def updateMotorSpeeds(self):
        right_side_speed = (self.des_lin_speed 
            + HALF_WHEEL_BASE * self.des_ang_speed)
        left_side_speed = (self.des_lin_speed 
            - HALF_WHEEL_BASE * self.des_ang_speed)

        right_motor_speed = right_side_speed / WHEEL_RADIUS
        left_motor_speed = left_side_speed / WHEEL_RADIUS

        if left_motor_speed > MAX_WHEEL_SPEED:
            self._node.get_logger().warning(
                "Left motor speed {} exceeds threshold {}".format(
                    left_motor_speed, MAX_WHEEL_SPEED
                )
            )
            
            self._node.get_logger().warning(
                "Right motor speed {} exceeds threshold {}".format(
                    right_motor_speed, MAX_WHEEL_SPEED
                )
            )

        for motor in self.left_motors:
            motor.setVelocity(left_motor_speed)
        
        for motor in self.right_motors:
            motor.setVelocity(right_motor_speed)

        pass
    
    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        self.updateMotorSpeeds()
        pass
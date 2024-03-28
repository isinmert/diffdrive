import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TwistStamped
from diffdrive_msgs.msg import Pose2DStamped, Pose2DTrajectory
from diffdrive_msgs.srv import Pose2DTrajSrv
from .tracker_utils import (PDController, compareTime, timeDiff, timeToFloat, 
                            getRotationMatrix)
from .base_tracker import TrajectoryTracker

import numpy as np

class FeedbackLinearizeTracker(TrajectoryTracker):
    """
    Trajectory tracker for differential drive robot. 
    Takes a trajectory and outputs desired longitudinal and angular velocity 
    for the vehicle
    """
    def __init__(self):
        super().__init__("feedback_linearize_tracker")

        # Initialize pd controllers
        Kp_x = 0.7
        Kp_y = 0.7
        Kd_x = 1.0
        Kd_y = 1.0
        self.PD_x = PDController("x_controller", kp=Kp_x, kd=Kd_x)
        self.PD_y = PDController("y_controller", kp=Kp_y, kd=Kd_y)

        pass

    
    def cmd_vel_pub_callback(self):
        cmd_vel_msg = Twist()
        car_state_time = self.car_state.timestamp
        car_state_x  = self.car_state.pose.x
        car_state_y = self.car_state.pose.y
        car_state_theta = self.car_state.pose.theta

        rot_matrix = getRotationMatrix(car_state_theta)
        DEBUG = self.get_parameter("debug").value
        DEBUG_SEC = self.get_parameter("debug_sec").value
        SAFETY_THRESHOLD = self.get_parameter("safety_threshold").value

        # check the difference between the current time and last state time
        cur_time = timeToFloat(self.get_clock().now().to_msg())
        if abs(cur_time-timeToFloat(car_state_time)) >= SAFETY_THRESHOLD:
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            if DEBUG:
                self.get_logger().info("State is not observed!!", 
                                throttle_duration_sec=DEBUG_SEC)
            return

        # Get Pos, vel and acceleration values from trajectory 
        success, state_at_t = self._get_state_from_traj(car_state_time)
        
        if success:
            self.get_logger().info("Trajectory is being tracked",
                                   throttle_duration_sec=DEBUG_SEC)
        else:
            return
        
        if DEBUG:
            self.get_logger().info("theta : {}".format(car_state_theta), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("rotation_matrix: {}".format(rot_matrix), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info(
                "inv_rot_mat: {}".format(np.linalg.inv(rot_matrix)), 
                throttle_duration_sec=DEBUG_SEC)

        # Compute error values
        err_x = state_at_t[0] - car_state_x
        err_y = state_at_t[1] - car_state_y

        # check whether the goal is reached
        if (timeToFloat(car_state_time) > self.spline_x.x[-1] 
            and abs(err_x) <= 1.0e-1
            and abs(err_y) <= 1.0e-1):
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.prev_cmd_vel.twist = cmd_vel_msg
            self.prev_cmd_vel.header.stamp = self.clock.now().to_msg()
            self.get_logger().info("Trajectory is completed!!!",
                                   throttle_duration_sec=DEBUG_SEC)
            self.trajectory_set = False
            self.trajectory = Pose2DTrajectory()
            self.spline_x = None
            self.spline_y = None
            self.spline_theta = None
            return

        if DEBUG:
            self.get_logger().info("traj_x : {}".format(state_at_t[0]), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("traj_y : {}".format(state_at_t[1]), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("car_state_x : {}".format(car_state_x), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("car_state_y : {}".format(car_state_y), 
                                   throttle_duration_sec=DEBUG_SEC)
        
        self.get_logger().info("err_x : {}".format(err_x), 
                                throttle_duration_sec=DEBUG_SEC)
        self.get_logger().info("err_y : {}".format(err_y), 
                                throttle_duration_sec=DEBUG_SEC)

        # Compute u_x and u_y 
        PD_x_output = self.PD_x.getOutput(err_x, car_state_time)
        PD_y_output = self.PD_y.getOutput(err_y, car_state_time)
        u_x = state_at_t[4] + PD_x_output
        u_y = state_at_t[5] + PD_y_output

        if DEBUG:
            self.get_logger().info("pd_x_output : {}".format(PD_x_output), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("pd_y_output : {}".format(PD_y_output), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("ax : {}".format(state_at_t[4]),
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("ay : {}".format(state_at_t[5]),
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("u_x : {}".format(u_x),
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("u_y : {}".format(u_y),
                                   throttle_duration_sec=DEBUG_SEC)


        # Compute vdot and vw values 
        dummy = np.linalg.inv(rot_matrix) @ np.array([u_x, u_y])
        vdot = dummy[0]
        v_times_w = dummy[1]
        if DEBUG:
            self.get_logger().info("vdot : {}".format(vdot), 
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("v_times_w : {}".format(v_times_w), 
                                   throttle_duration_sec=DEBUG_SEC)

        # Compute vk and wk 
        EPS = 1e-3
        EPS_dt = 0.2
        cur_time = self.clock.now().to_msg()
        dt_duration = timeDiff(cur_time, self.prev_cmd_vel.header.stamp)
        dt = timeToFloat(dt_duration)
        vkm1 = self.prev_cmd_vel.twist.linear.x
        if dt <= EPS_dt:
            vk = vkm1 + vdot * dt
        else:
            vk = 0.0
        if vk >= EPS or vk <= -EPS:
            wk = v_times_w/vk
        else:
            wk = 0.0

        if DEBUG:
            self.get_logger().info("dt : {}".format(dt),
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("vkm1 : {}".format(vkm1),
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("vk : {}".format(vk),
                                   throttle_duration_sec=DEBUG_SEC)
            self.get_logger().info("wk : {}".format(wk),
                                   throttle_duration_sec=DEBUG_SEC)
        
        # write cmd_vel message 
        cmd_vel_msg.linear.x = vk
        cmd_vel_msg.angular.z = wk

        # save previous cmd_vel
        self.prev_cmd_vel.twist = cmd_vel_msg
        self.prev_cmd_vel.header.stamp = self.clock.now().to_msg()

        # Publish message
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        return
    

def main():
    # Initialize ros client
    rclpy.init()

    # Initialize a trajectory tracker node
    traj_tracker = FeedbackLinearizeTracker()

    try:
        # spin
        rclpy.spin(traj_tracker)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:  
        # Destroy node on shutdown
        traj_tracker.destroy_node()
        # Shutdown ros client
        rclpy.try_shutdown()
    pass

if __name__ == "__main__":
    main()
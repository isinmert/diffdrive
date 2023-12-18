import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time
from diffdrive_msgs.msg import Pose2DStamped, Pose2DTrajectory
from diffdrive_msgs.srv import Pose2DTrajSrv
from .tracker_utils import (PDController, compareTime, timeDiff, timeToFloat, 
                            getRotationMatrix)


from typing import Tuple, List

import numpy as np
from scipy.interpolate import CubicSpline

class TrajectoryTracker(Node):
    """
    Trajectory tracker for differential drive robot. 
    Takes a trajectory and outputs desired longitudinal and angular velocity 
    for the vehicle
    """
    def __init__(self, name:str=""):
        super().__init__(name)

        self.declare_parameter("robot_name", "")
        self.declare_parameter("check_step", 1)
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_sec", 0.5)
        self.declare_parameter("safety_threshold", 0.2)
        self.declare_parameter("cmd_vel_pub_rate", 0.05)
        
        self.save_traj_srv = self.create_service(
            Pose2DTrajSrv, 
            self.get_parameter("robot_name").value+"/trajectory", 
            self.save_traj_srv_callback
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            self.get_parameter("robot_name").value + "/cmd_vel",
            1
        )

        self.create_subscription(
            Pose2DStamped, 
            self.get_parameter("robot_name").value + "/state",
            self.state_callback, 
            qos_profile=1)
        
        # save last car state received
        self.car_state = Pose2DStamped()
        # save last unique trajectory received
        self.trajectory = Pose2DTrajectory()
        # Initialize splines to store the trajectory
        self.spline_x = None
        self.spline_y = None
        self.spline_theta = None
        self.trajectory_set = False
        self.prev_cmd_vel = TwistStamped()

        self.clock = self.get_clock()

        # Create safety timer
        self.create_timer(
            self.get_parameter("check_step").value, 
            self.timer_callback
            )
        
        # create timer to update cmd_vel update
        self.create_timer(
            self.get_parameter("cmd_vel_pub_rate").value,
            self.cmd_vel_pub_callback
        )

        self.check_trajectory_srv = self.create_service(
            Trigger,
            self.get_parameter("robot_name").value + "/check_traj",
            self.check_traj_callback
            )

        pass

    def save_traj_srv_callback(self, request:Pose2DTrajSrv.Request, 
                               response:Pose2DTrajSrv.Response):
        
        if len(request.trajectory.poses) < 3:
            response.success.data = False
            self.get_logger().warning("trajectory should have at least 3 poses.")
            return response
        
        self.trajectory = request.trajectory

        t0 = request.trajectory.header.stamp
        t0_float = timeToFloat(t0)
        t_list = []
        x_list = []
        y_list = []
        theta_list = []

        for stamped_pose in request.trajectory.poses:
            t = stamped_pose.timestamp
            t_list.append(t0_float + timeToFloat(t))
            x_list.append(stamped_pose.pose.x)
            y_list.append(stamped_pose.pose.y)
            theta_list.append(stamped_pose.pose.theta)
            
        self.spline_x = CubicSpline(t_list, x_list)
        self.spline_y = CubicSpline(t_list, y_list)
        self.spline_theta = CubicSpline(t_list, theta_list)
    
        self.trajectory_set = True
        self.get_logger().info("trajectory is UPDATED!!")

        response.success.data = True
        
        return response

    def check_traj_callback(self, request:Trigger.Request, 
                            response:Trigger.Response):
        if self.trajectory_set:
            response.success = True
            response.message = ("Trajectory header : {}"
                                .format(self.trajectory.header.frame_id))
        else:
            response.success = False
            response.message = ("Trajectory is not set.")
        return response

    def trajectory_callback(self, trj_msg:Pose2DTrajectory)->None:
        """
        Save observed trajectory if it's frame_id is different than the
        saved trajectory.
        """
        if trj_msg.header.frame_id != self.trajectory.header.frame_id:
            self.trajectory = trj_msg
            # Update or generate trajectory splines
            t0 = trj_msg.header.stamp
            t0_float = timeToFloat(t0)
            t_list = []
            x_list = []
            y_list = []
            theta_list = []
            
            for stamped_pose in trj_msg.poses:
                t = stamped_pose.timestamp
                t_list.append(t0_float + timeToFloat(t))
                x_list.append(stamped_pose.pose.x)
                y_list.append(stamped_pose.pose.y)
                theta_list.append(stamped_pose.pose.theta)
            
            self.spline_x = CubicSpline(t_list, x_list)
            self.spline_y = CubicSpline(t_list, y_list)
            self.spline_theta = CubicSpline(t_list, theta_list)
        
            self.trajectory_set = True
            self.get_logger().info("trajectory is UPDATED!!")
        else:
            pass
        return
    
    def timer_callback(self) -> None:
        if self.trajectory_set:
            cur_time = self.clock.now().to_msg()
            prev_cmd_time = self.prev_cmd_vel.header.stamp
            time_diff = timeDiff(cur_time, prev_cmd_time)
            time_diff_float = timeToFloat(time_diff)
            if time_diff_float >= self.get_parameter("check_step").value:
                safety_msg = Twist()
                self.cmd_vel_publisher.publish(safety_msg)
                self.get_logger().info("Stopping message sent")

        return
    
    def _get_state_from_traj(self, t:Time) -> Tuple[bool, List[float]]:
        """
        Get position, speed and acceleration values at a timestamp from 
        the saved trajectory.
        """
        res = [0.0 for _ in range(6)]
        success = False
        DEBUG = self.get_parameter("debug").value
        DEBUG_SEC = self.get_parameter("debug_sec").value
        if DEBUG:
            logger = self.get_logger()
        
        if self.trajectory_set:
            pass
        else:
            if DEBUG:
                logger.info("Trajectory is not set!!", 
                            throttle_duration_sec=DEBUG_SEC)
            return success, res
        
        t_float = timeToFloat(t)

        # check whether t_float is between 
        t0_traj, tf_traj = self.spline_x.x[0], self.spline_x.x[-1]
        t_between = t0_traj <= t_float <= tf_traj
        if t_between:
            success = True
            if DEBUG: 
                logger.info(
                    "t:{} is between t0:{} and tf:{}"
                        .format(t_float, t0_traj, tf_traj),
                        throttle_duration_sec=DEBUG_SEC
                    )
        elif t0_traj > t_float:
            success = True
            if DEBUG:
                logger.info(
                    "t:{} is lower than t0:{}".format(t_float, t0_traj),
                    throttle_duration_sec=DEBUG_SEC
                )
            t_float = t0_traj
        else:
            success = True
            if DEBUG:
                logger.info(
                    "t:{} is greater than tf:{}".format(t_float, tf_traj),
                    throttle_duration_sec=DEBUG_SEC
                )
            t_float = tf_traj

        res[0] = self.spline_x(t_float)
        res[1] = self.spline_y(t_float)
        # if t_float is not valid take the velocities and accelerations as 0
        if t_between:
            res[2] = self.spline_x.derivative(1)(t_float)
            res[3] = self.spline_y.derivative(1)(t_float)
            res[4] = self.spline_x.derivative(2)(t_float)
            res[5] = self.spline_y.derivative(2)(t_float)

        return success, res

    
    def cmd_vel_pub_callback(self):
        """
        Implement this function in children classes.
        """
        raise NotImplementedError("Implement this in the children classes.")
    
    
    def state_callback(self, msg:Pose2DStamped):
        """
        Save last observed car state
        """
        self.car_state = msg
        DEBUG = self.get_parameter("debug").value
        DEBUG_SEC = 0.5
        if DEBUG:
            self.get_logger().info("car state is obtained", 
                                   throttle_duration_sec=DEBUG_SEC)        
        return

def main():
    pass

if __name__ == "__main__":
    main()
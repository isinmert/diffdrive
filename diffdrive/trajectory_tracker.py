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
    def __init__(self):
        super().__init__("trajectory_tracker")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("check_step", 1)
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_sec", 0.5)
        self.declare_parameter("safety_threshold", 0.2)
        self.declare_parameter("cmd_vel_pub_rate", 0.05)

        # self.create_subscription(Pose2DTrajectory,
        #     self.get_parameter("robot_name").value+"/trajectory", 
        #     self.trajectory_callback, 
        #     qos_profile=1)
        
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

        # Initialize pd controllers
        Kp_x = 0.7
        Kp_y = 0.7
        Kd_x = 1.0
        Kd_y = 1.0
        self.PD_x = PDController("x_controller", kp=Kp_x, kd=Kd_x)
        self.PD_y = PDController("y_controller", kp=Kp_y, kd=Kd_y)

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
                self.get_logger("State is not observed!!", 
                                throttle_duration_sec=DEBUG_SEC)
            return

        # Get Pos, vel and acceleration values from trajectory 
        success, state_at_t = self._get_state_from_traj(car_state_time)
        
        if success == True:
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
    # Initialize ros client
    rclpy.init()

    # Initialize a trajectory tracker node
    traj_tracker = TrajectoryTracker()

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
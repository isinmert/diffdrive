import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time
from diffdrive_msgs.msg import Pose2DStamped
from .tracker_utils import (PDController, compareTime, timeDiff, timeToFloat, 
                            getRotationMatrix)


from typing import Tuple, List

import numpy as np

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

        self.create_subscription(JointTrajectory,
            self.get_parameter("robot_name").value+"/trajectory", 
            self.trajectory_callback, 
            qos_profile=1)
        
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
        
        self.trajectory = JointTrajectory()
        self.trajectory_set = False

        # Initialize pd controllers
        self.PD_x = PDController("x_controller", kp=0.2, kd=1.0)
        self.PD_y = PDController("y_controller", kp=0.2, kd=1.0)

        self.prev_cmd_vel = TwistStamped()

        self.clock = self.get_clock()

        # Create safety timer
        self.create_timer(
            self.get_parameter("check_step").value, 
            self.timer_callback
            )

        pass

    def trajectory_callback(self, trj_msg:JointTrajectory)->None:
        """
        Save observed trajectory if it's frame_id is different than the
        saved trajectory.
        """
        if trj_msg.header.frame_id != self.trajectory.header.frame_id:
            self.trajectory = trj_msg
        self.trajectory_set = True
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
    
    def _get_state_from_traj(self, t:Time, traj:JointTrajectory) \
        -> Tuple[bool, List[float]]:
        """
        Get state and acceleration values at a timestep from a given 
        JointTrajectory.
        """

        res = [0.0 for _ in range(6)]
        success = False
        DEBUG = self.get_parameter("debug").value
        DEBUG_SEC = 0.5
        if DEBUG:
            logger = self.get_logger()

        if self.trajectory_set:
            pass
        else:
            # self.get_logger().info("Trajectory is not set")
            pass
            return success, res

        traj_points = traj.points
        t0_traj = traj.header.stamp
        if len(traj_points) == 0:
            self.get_logger().warning("Trajectory is empty!!")
            return success, res
        elif len(traj_points) == 1:
            self.get_logger().warning("Num traj_points = 1. Not sufficient!!")
            return success, res
        
        if DEBUG:
            logger.info("t0_traj_sec : {}".format(t0_traj.sec), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t0_traj_nano : {}".format(t0_traj.nanosec),
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_sec : {}".format(t.sec), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_nano : {}".format(t.nanosec), 
                        throttle_duration_sec=DEBUG_SEC)

        t_traj_final = traj_points[-1].time_from_start
        t_greater_t0 = compareTime(t, t0_traj)
        if t_greater_t0:
            t_rel = timeDiff(t, t0_traj)
        else:
            t_rel = Time()
        t_less_t_final =  compareTime(t_traj_final, t_rel)
        if t_less_t_final:
            pass
        else:
            t_rel = t_traj_final

        for k in range(len(traj_points)-1):
            t_l = traj_points[k].time_from_start
            t_r = traj_points[k+1].time_from_start
            if compareTime(t_rel, t_l) and compareTime(t_r, t_rel):
                break
        
        success = True
        dt_time = timeDiff(t_rel, t_l)
        dt_total_time = timeDiff(t_r, t_l)
        dt = timeToFloat(dt_time)
        dt_total = timeToFloat(dt_total_time)

        pxk = traj_points[k].positions[0]
        pyk = traj_points[k].positions[1]
        vxk = traj_points[k].velocities[0]
        vyk = traj_points[k].velocities[1]
        axk = traj_points[k].accelerations[0]
        ayk = traj_points[k].accelerations[1]

        pxkp1 = traj_points[k+1].positions[0]
        pykp1 = traj_points[k+1].positions[1]
        vxkp1 = traj_points[k+1].velocities[0]
        vykp1 = traj_points[k+1].velocities[1]
        axkp1 = traj_points[k+1].accelerations[0]
        aykp1 = traj_points[k+1].accelerations[1]

        ay_diff = aykp1 - ayk
        ax_diff = axkp1 - axk

        j_x = ax_diff / dt_total
        j_y = ay_diff / dt_total

        px_cur = pxk + vxk*dt + axk*(dt**2)*0.5 + ((dt**3)/6) * j_x
        py_cur = pyk + vyk*dt + ayk*(dt**2)*0.5 + ((dt**3)/6) * j_y
        vx_cur = vxk + axk*dt + (dt**2) * 0.5 * j_x
        vy_cur = vyk + ayk*dt + (dt**2) * 0.5 * j_y
        ax_cur = axk + dt * j_x
        ay_cur = ayk + dt * j_y

        if DEBUG:
            logger.info("k : {}".format(k),
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_l_sec : {}".format(t_l.sec),
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_l_nano : {}".format(t_l.nanosec),
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_r_sec : {}".format(t_r.sec),
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_r_nano : {}".format(t_r.nanosec),
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_rel_sec : {}".format(t_rel.sec), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("t_rel_nano : {}".format(t_rel.nanosec), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("px_cur : {}".format(px_cur), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("py_cur : {}".format(py_cur), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("pxk : {}".format(pxk), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("pyk : {}".format(pyk), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("vxk : {}".format(vxk), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("vyk : {}".format(vyk), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("axk : {}".format(axk), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("ayk : {}".format(ayk), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("j_x : {}".format(j_x), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("j_y : {}".format(j_y), 
                        throttle_duration_sec=DEBUG_SEC)

        res = [px_cur, py_cur, vx_cur, vy_cur, ax_cur, ay_cur]

        return success, res
    
    def state_callback(self, msg:Pose2DStamped):
        cmd_vel_msg = Twist()
        car_state_time = msg.timestamp
        car_state_x  = msg.pose.x
        car_state_y = msg.pose.y
        car_state_theta = msg.pose.theta

        rot_matrix = getRotationMatrix(car_state_theta)
        DEBUG = self.get_parameter("debug").value
        DEBUG_SEC = 0.5

        # Get Pos, vel and acceleration values from trajectory 
        success, state_at_t = self._get_state_from_traj(car_state_time, 
                                                        self.trajectory)
        
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
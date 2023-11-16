import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time
from racecar_msgs.msg import CarState
from .tracker_utils import (PDController, compareTime, timeDiff, timeToFloat, 
                            getRotationMatrix)

from typing import Tuple, List

import numpy as np

class LyapunovTracker(Node):
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

        self.declare_parameter("k_x", 0.2)
        self.declare_parameter("k_y", 0.2)
        self.declare_parameter("k_theta", 0.2)

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
            CarState, 
            self.get_parameter("robot_name").value + "/state",
            self.state_callback, 
            qos_profile=1)
        
        self.trajectory = JointTrajectory()
        self.trajectory_set = False

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
        terminal = False
        DEBUG = self.get_parameter("debug").value
        if DEBUG:
            logger = self.get_logger()

        if self.trajectory_set:
            pass
        else:
            # self.get_logger().info("Trajectory is not set")
            return success, terminal, res

        traj_points = traj.points
        t0_traj = traj.header.stamp
        if len(traj_points) == 0:
            self.get_logger().warning("Trajectory is empty!!")
            return success, res
        elif len(traj_points) == 1:
            self.get_logger().warning("Num traj_points = 1. Not sufficient!!")
            return success, res
        
        if DEBUG:
            logger.info("t0_traj_sec : {}".format(t0_traj.sec))
            logger.info("t0_traj_nano : {}".format(t0_traj.nanosec))
            logger.info("t_sec : {}".format(t.sec))
            logger.info("t_nano : {}".format(t.nanosec))

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
            terminal = True

        if terminal:
            px_cur = traj_points[-1].positions[0]
            py_cur = traj_points[-1].positions[1]
            vx_cur = traj_points[-1].velocities[0]
            vy_cur = traj_points[-1].velocities[1]
            ax_cur = traj_points[-1].accelerations[0]
            ay_cur = traj_points[-1].accelerations[1]
        else:
            for k in range(len(traj_points)-1):
                t_l = traj_points[k].time_from_start
                t_r = traj_points[k+1].time_from_start
                if compareTime(t_rel, t_l) and compareTime(t_r, t_rel):
                    break
        
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
                logger.info("k : {}".format(k))
                logger.info("t_l_sec : {}".format(t_l.sec))
                logger.info("t_l_nano : {}".format(t_l.nanosec))
                logger.info("t_r_sec : {}".format(t_r.sec))
                logger.info("t_r_nano : {}".format(t_r.nanosec))
                logger.info("t_rel_sec : {}".format(t_rel.sec))
                logger.info("t_rel_nano : {}".format(t_rel.nanosec))
                logger.info("px_cur : {}".format(px_cur))
                logger.info("py_cur : {}".format(py_cur))
                logger.info("pxk : {}".format(pxk))
                logger.info("pyk : {}".format(pyk))
                logger.info("vxk : {}".format(vxk))
                logger.info("vyk : {}".format(vyk))
                logger.info("axk : {}".format(axk))
                logger.info("ayk : {}".format(ayk))
                logger.info("j_x : {}".format(j_x))
                logger.info("j_y : {}".format(j_y))


        res = [px_cur, py_cur, vx_cur, vy_cur, ax_cur, ay_cur]
        success = True
        return success, terminal, res
    
    def state_callback(self, msg:CarState):

        # Extract position and orientation of the robot from the state message
        cmd_vel_msg = Twist()
        car_state_time = msg.timestamp
        car_state_x  = msg.x
        car_state_y = msg.y
        car_state_theta = msg.theta

        # extract the controller parameters
        k_x = self.get_parameter("k_x").value
        k_y = self.get_parameter("k_y").value
        k_theta = self.get_parameter("k_theta").value

        rot_matrix = getRotationMatrix(car_state_theta)
        DEBUG = self.get_parameter("debug").value
        if DEBUG:
            logger = self.get_logger()

        # Get Pos, vel and acceleration values from trajectory 
        success, terminal, state_at_t = self._get_state_from_traj(
            car_state_time, self.trajectory
            )
        
        if success:
            self.get_logger().info("Trajectory is being tracked")
        else:
            return
        
        if DEBUG:
            logger.info("theta : {}".format(car_state_theta))
            logger.info("rotation_matrix: {}".format(rot_matrix))
            logger.info("inv_rot_mat: {}".format(np.linalg.inv(rot_matrix)))

        # Get q_t, q_ref_t and compute q_err
        q_ref_t = np.zeros((3, 1))
        q_t = np.zeros((3, 1))
        q_t[0] = car_state_x
        q_t[1] = car_state_y
        q_t[2] = car_state_theta
        q_ref_t[0] = state_at_t[0]
        q_ref_t[1] = state_at_t[1]
        vk_x_ref, vk_y_ref = state_at_t[2], state_at_t[3]
        thetak_ref = np.arctan2(vk_y_ref, vk_x_ref)
        q_ref_t[2] = thetak_ref
        q_err_t = q_ref_t - q_t
        if DEBUG:
            logger.info("px_car : {}".format(car_state_x))
            logger.info("py_car : {}".format(car_state_y))
            logger.info("theta_car : {}".format(car_state_theta))
            logger.info("px_ref : {}".format(q_ref_t[0]))
            logger.info("py_ref : {}".format(q_ref_t[1]))
            logger.info("theta_ref : {}".format(thetak_ref))

        # Compute error terms err_x, err_y, err_theta
        helper_matrix = np.eye(3)
        helper_matrix[0:2, 0:2] = getRotationMatrix(car_state_theta).T
        err_vec = helper_matrix @ q_err_t
        err_vec = err_vec.flatten()
        err_x, err_y, err_theta = err_vec[0], err_vec[1], err_vec[2]
        if DEBUG:
            logger.info("err_x : {}".format(err_x))
            logger.info("err_y : {}".format(err_y))
            logger.info("err_theta : {}".format(err_theta))

        # compute reference traj linear and angular velocity
        vk_ref = np.sqrt(vk_x_ref**2 + vk_y_ref**2)
        ak_x_ref, ak_y_ref = state_at_t[4], state_at_t[5]
        ak_ref_vec = np.array([ak_x_ref, ak_y_ref])
        vk_ref_vec = np.array([vk_x_ref, vk_y_ref])
        tk_ref_vec = vk_ref_vec / vk_ref

        ak_lon_magnitude = np.dot(ak_ref_vec, tk_ref_vec)
        ak_lon_vec = ak_lon_magnitude * tk_ref_vec 
        ak_lat_vec = ak_ref_vec - ak_lon_vec
        ak_lat_magnitude = np.linalg.norm(ak_lat_vec, 2)

        EPS_SPEED = 1e-2
        if abs(vk_ref) >= EPS_SPEED:
            wk_magnitude = np.sqrt(abs(ak_lat_magnitude/vk_ref))
            wk_ref = np.sign(np.cross(tk_ref_vec, ak_lat_vec)) * wk_magnitude 
        else:
            wk_ref = 0.

        if DEBUG:
            logger.info("vk_ref : {}".format(vk_ref))
            logger.info("wk_ref : {}".format(wk_ref))

        vk_fb = k_x * err_x
        helper_sin_err_theta = 1.0 # sin(err_theta)/err_theta
        EPS_THETA = 1e-1
        if abs(err_theta) >= EPS_THETA:
            helper_sin_err_theta = np.sin(err_theta)/err_theta
        wk_fb = (k_y * vk_ref * helper_sin_err_theta * err_y 
                   + (k_theta * err_theta))
        
        vk = (vk_ref * np.cos(err_theta)) + vk_fb
        wk = wk_ref + wk_fb

        if terminal and max(err_x, err_y) <= 1e-1:
            vk = 0.0
            wk = 0.0
            if DEBUG:
                logger.info("goal is reached!!")

        if DEBUG:
            logger.info("vk : {}".format(vk))
            logger.info("wk : {}".format(wk))

        cmd_vel_msg.linear.x = vk
        cmd_vel_msg.angular.z = wk
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        return

def main():
    # Initialize ros client
    rclpy.init()

    # Initialize a trajectory tracker node
    tracker = LyapunovTracker()

    try:
        # spin
        rclpy.spin(tracker)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:  
        # Destroy node on shutdown
        tracker.destroy_node()
        # Shutdown ros client
        rclpy.try_shutdown()
    pass

if __name__ == "__main__":
    main()
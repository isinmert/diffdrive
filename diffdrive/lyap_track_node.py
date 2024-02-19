import rclpy
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TwistStamped
from diffdrive_msgs.msg import Pose2DStamped, Pose2DTrajectory
from .tracker_utils import compareTime, timeDiff, timeToFloat, getRotationMatrix
from .base_tracker import TrajectoryTracker

import numpy as np

class LyapunovTracker(TrajectoryTracker):
    """
    Trajectory tracker for differential drive robot. 
    Takes a trajectory and outputs desired longitudinal and angular velocity 
    for the vehicle
    """
    def __init__(self):
        super().__init__("lyapunov_tracker")

        # Define tracker specific parameters
        self.declare_parameter("k_x", 1.0)
        self.declare_parameter("k_y", 1.0)
        self.declare_parameter("k_theta", 1.0)
        return

    
    def cmd_vel_pub_callback(self):
        """
        cmd_vel message generation for lypanunov trajectory tracking controller.
        """
        cmd_vel_msg = Twist()
        car_state_time = self.car_state.timestamp
        car_state_x = self.car_state.pose.x
        car_state_y = self.car_state.pose.y
        car_state_theta = self.car_state.pose.theta

        logger = self.get_logger()

        k_x = self.get_parameter("k_x").value
        k_y = self.get_parameter("k_y").value
        k_theta = self.get_parameter("k_theta").value

        rot_matrix = getRotationMatrix(car_state_theta)
        DEBUG = self.get_parameter("debug").value
        DEBUG_SEC = self.get_parameter("debug_sec").value
        SAFETY_THRESHOLD = self.get_parameter("safety_threshold").value

        # check the difference between the current time and last state time
        cur_time = timeToFloat(self.get_clock().now().to_msg())
        if abs(cur_time-timeToFloat(car_state_time)) >= SAFETY_THRESHOLD:
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            if DEBUG:
                logger.info("State is not observed!!", 
                            throttle_duration_sec=DEBUG_SEC)
            return

        # Get Pos, vel and acceleration values from trajectory 
        success, state_at_t = self._get_state_from_traj(car_state_time)

        if success == True:
            logger.info("Trajectory is being tracked",
                        throttle_duration_sec=DEBUG_SEC)
        else:
            return
        
        if DEBUG:
            logger.info("theta : {}".format(car_state_theta), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("rotation_matrix: {}".format(rot_matrix), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("inv_rot_mat: {}".format(np.linalg.inv(rot_matrix)), 
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
            logger.info("Trajectory is completed!!!",
                        throttle_duration_sec=DEBUG_SEC)
            self.trajectory_set = False
            self.trajectory = Pose2DTrajectory()
            self.spline_x = None
            self.spline_y = None
            self.spline_theta = None
            return

        if DEBUG:
            logger.info("traj_x : {}".format(state_at_t[0]), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("traj_y : {}".format(state_at_t[1]), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("car_state_x : {}".format(car_state_x), 
                        throttle_duration_sec=DEBUG_SEC)
            logger.info("car_state_y : {}".format(car_state_y), 
                        throttle_duration_sec=DEBUG_SEC)
        
        logger.info("err_x : {}".format(err_x), 
                    throttle_duration_sec=DEBUG_SEC)
        logger.info("err_y : {}".format(err_y), 
                    throttle_duration_sec=DEBUG_SEC)
        
        # Compute cmd_vel
        # Get q_t, q_ref_t and compute q_err
        q_ref_t = np.zeros((3, 1))
        q_t = np.zeros((3, 1))
        q_t[0] = car_state_x
        q_t[1] = car_state_y
        q_t[2] = car_state_theta
        q_ref_t[0] = state_at_t[0]
        q_ref_t[1] = state_at_t[1]
        vxk_ref, vyk_ref = state_at_t[2], state_at_t[3]
        if(timeToFloat(car_state_time) >= self.spline_theta.x[-1]):
            t_float = self.spline_theta.x[-1]
        else:
            t_float = timeToFloat(car_state_time)
        thetak_ref = self.spline_theta(t_float)
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
        vk_ref = np.sqrt(vxk_ref**2 + vyk_ref**2)
        ak_x_ref, ak_y_ref = state_at_t[4], state_at_t[5]
        ak_ref_vec = np.array([ak_x_ref, ak_y_ref])
        vk_ref_vec = np.array([vxk_ref, vyk_ref])
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
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from diffdrive_msgs.msg import Pose2DStamped, Pose2DTrajectory
from diffdrive_msgs.srv import Pose2DSrv, Pose2DTrajSrv, CircTraj
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from builtin_interfaces.msg import Time

from .traj_gen import trajectoryGenerator, generateCircTraj
from .tracker_utils import getRotationMatrix, FloatToTime

import numpy as np

class trajGenNode(Node):
    def __init__(self):
        super().__init__("trajectory_generator")

        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("robot_name", "")
        self.declare_parameter("dt", 0.5)
        self.declare_parameter("N", 20)

        self.p_0 = float()
        self.p_d = float()
        self.theta_0 = float()
        self.theta_d = float()
        self.v_0 = float()
        self.t_init = float()
        self.t_init_time = Time()

        self.counter = 0
        timer_step = 0.05 # in seconds
        self.timer = self.create_timer(timer_step, self.timer_callback)

        self.trajectory = Pose2DTrajectory()
        self.initial_state = Pose2DStamped()
        self.initial_state_set = False
        self.desired_state_set = False
        self.trajectory_set = False
        self.trajGen = trajectoryGenerator(
            self.get_parameter("N").value,
            self.get_parameter("dt").value, 
            self.get_parameter("max_speed").value
            )

        self.robot_name = self.get_parameter("robot_name").value
        
        self.create_subscription(Pose2DStamped, 
                                 self.robot_name+"/state", 
                                 self.state_callback, 
                                 qos_profile=1)
        
        self.traj_pub_cli = self.create_client(
            Pose2DTrajSrv,
            self.robot_name+'/trajectory'
            )

        self.clear_traj_srv = self.create_service(
            Empty, 
            self.robot_name+"/clear_traj", 
            self.clear_traj_callback
            )
        
        self.desired_pose_srv = self.create_service(
            Pose2DSrv, 
            self.robot_name+"/desired_pose",
            self.des_pose_callback
        )

        self.circ_traj_srv = self.create_service(
            CircTraj, 
            self.robot_name+"/get_circ_traj", 
            self.circ_traj_callback
        )
        
        self.clock = self.get_clock()
        
    pass

    def clear_traj_callback(self, request:Empty.Request, 
                            response:Empty.Response):
        """
        Once clear traj service is requested clear the trajectory and 
        unset initial and desired states
        """
        self.trajectory.points.clear()
        self.trajectory_set = False
        self.initial_state_set = False
        self.desired_state_set = False
        self.get_logger().info("Trajectory is cleared!!")
        
        return response

    def state_callback(self, msg:Pose2DStamped) -> None:
        """
        Save the current robot state as the initial state
        """
        self.initial_state.timestamp = msg.timestamp
        self.initial_state.pose.x = msg.pose.x
        self.initial_state.pose.y = msg.pose.y
        self.initial_state.pose.theta = msg.pose.theta
        self.initial_state_set = True

        self.p_0 = np.zeros(2)
        self.theta_0 = 0.
        self.theta_0 = msg.pose.theta
        self.p_0[0] = msg.pose.x
        self.p_0[1] = msg.pose.y
        self.t_init_time = msg.timestamp
        self.t_init = self.t_init_time.sec + (self.t_init_time.nanosec/1e9)
        return

    def des_pose_callback(self, request:Pose2D, response) -> None:
        self.p_d = np.zeros(2)
        self.theta_d = 0.
        self.p_d[0] = request.pose.x
        self.p_d[1] = request.pose.y
        self.theta_d = request.pose.theta
        self.desired_state_set = True
        self.get_logger().info("Desired pose is saved")
        response.success = Bool()
        response.success.data = True
        return response
    
    def circ_traj_callback(self, request:CircTraj.Request, 
                           response:CircTraj.Response):
        """
        Generate a circular trajectory and call trajectory service.
        """
        if not self.initial_state_set:
            response.success = Bool()
            response.success.data = False
            self.get_logger().warning("Initial state is not set!")
            return response
        
        x0 = self.initial_state.pose.x
        y0 = self.initial_state.pose.y
        theta_0 = self.initial_state.pose.theta

        angle_req = request.angle
        rad_req = request.radius
        CW = request.cw.data

        trajectory = Pose2DTrajectory()
        trajectory.header.stamp = self.clock.now().to_msg()

        try: 
            traj_arr = generateCircTraj(x0, y0, theta_0, rad_req, angle_req, 
                                        CW, self.get_parameter("max_speed").value, 
                                        self.get_parameter("dt").value)
            
            for k in range(traj_arr.shape[1]):
                traj_pose = Pose2DStamped()
                traj_pose.timestamp = FloatToTime(traj_arr[0, k])
                traj_pose.pose.x = traj_arr[1, k]
                traj_pose.pose.y = traj_arr[2, k]
                traj_pose.pose.theta = traj_arr[3, k]

                trajectory.poses.append(traj_pose)

            self.trajectory = trajectory
            self.trajectory_set = True
            response.success = Bool()
            response.success.data = True
            return response
        except:
            self.get_logger().warning("An error occured while generating trajectory.")
            self.trajectory_set = False
            response.success = Bool()
            response.success.data = False
            return response
    
    def solveProb(self) -> None:
        if self.initial_state_set and self.desired_state_set:
            init_state = np.vstack(
                [self.p_0[0], self.p_0[1], self.theta_0]
                )
            self.trajGen.setP0(init_state)
            
            des_state = np.vstack(
                [self.p_d[0], self.p_d[1], self.theta_d]
                )
            self.trajGen.setPd(des_state)

            self.trajGen.setProblem()
            self.trajGen.solveProblem()

            try:
                if self.trajGen.problem.status == 'optimal':
                    out_traj = self.trajGen.getTrajectory(t0=self.t_init)
                    px = out_traj[0, :]
                    py = out_traj[1, :]
                    vx = out_traj[2, :]
                    vy = out_traj[3, :]
                    ax = out_traj[4, :]
                    ay = out_traj[5, :]

                    self.trajectory.header.frame_id = ("trajectory" 
                                                    + str(self.counter))
                    self.trajectory.header.stamp = self.t_init_time
                    self.trajectory.poses.clear()

                    N = self.get_parameter("N").value
                    dt = self.get_parameter("dt").value
                    for k in range(N+1):
                        pose_stamped = Pose2DStamped()
                        time_from_start = Time()
                        time_from_start.sec = int(k*dt)
                        time_from_start.nanosec = int((k*dt - int(k*dt))*1.0e9)
                        pose_stamped.timestamp = time_from_start
                        pxk = px[k]
                        pyk = py[k]
                        vxk = vx[k]
                        vyk = vy[k]
                        axk = ax[k]
                        ayk = ay[k]

                        pose_stamped.pose.x = pxk
                        pose_stamped.pose.y = pyk
                        pose_stamped.pose.theta = np.arctan2(vyk, vxk)
                        self.trajectory.poses.append(pose_stamped)


                    self.trajectory_set = True
                    self.initial_state_set = False
                    self.desired_state_set = False
                    self.counter = self.counter + 1
                else:
                    self.get_logger().warning(
                        "Problem is not solved optimally"
                        )
            except:
                self.get_logger().error(
                    "Trajectory cannot be obtained from the trajGen"
                    )
        else:
            self.get_logger().warning("Set initial and desired state first")            

        return
    
    def timer_callback(self) -> None:
        if self.initial_state_set and self.desired_state_set:
            self.solveProb()
            self.get_logger().info("Problem is solved")
        if self.trajectory_set:
            # self.traj_publisher.publish(self.trajectory)
            # Now publish trajectory
            traj_req = Pose2DTrajSrv.Request()
            traj_req.trajectory = self.trajectory
            self.traj_pub_cli.call_async(traj_req)
            self.get_logger().info(
                "Trajectory {} is published".format(
                    self.trajectory.header.frame_id), 
                    throttle_duration_sec=1.0)
            self.initial_state_set = False
            self.desired_state_set = False
            self.trajectory_set = False

        return


def main(args=None):
    rclpy.init(args=args)

    traj_gen = trajGenNode()

    try:
        rclpy.spin(traj_gen)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        traj_gen.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main() 

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, Pose2D
from racecar_msgs.msg import CarState
from std_srvs.srv import Empty
from builtin_interfaces.msg import Duration, Time

from .traj_gen import trajectoryGenerator

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
        timer_step = 0.1 # in seconds
        self.timer = self.create_timer(timer_step, self.timer_callback)

        self.trajectory = JointTrajectory()
        self.initial_state = CarState()
        self.initial_state_set = False
        self.desired_state_set = False
        self.trajectory_set = False
        self.trajGen = trajectoryGenerator(
            self.get_parameter("N").value,
            self.get_parameter("dt").value, 
            self.get_parameter("max_speed").value
            )

        self.robot_name = self.get_parameter("robot_name").value
        self.create_subscription(Pose2D, 
                                 self.robot_name+"/desired_pose", 
                                 self.des_pose_callback, 
                                 qos_profile=1)
        self.create_subscription(CarState, 
                                 self.robot_name+"/state", 
                                 self.state_callback, 
                                 qos_profile=1)
        self.traj_publisher = self.create_publisher(
            JointTrajectory, 
            self.robot_name+"/trajectory", qos_profile=1
            )

        self.clear_traj_srv = self.create_service(
            Empty, 
            self.robot_name+"/clear_traj", 
            self.clear_traj_callback
            )
        
        self.clock = self.get_clock()
        
    pass

    def clear_traj_callback(self, request, response):
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

    def state_callback(self, msg:CarState) -> None:
        """
        Save the current robot state as the initial state
        """
        self.initial_state.timestamp = msg.timestamp
        self.initial_state.x = msg.x
        self.initial_state.y = msg.y
        self.initial_state.theta = msg.theta
        self.initial_state.xdot = msg.xdot
        self.initial_state.ydot = msg.ydot
        self.initial_state.thetadot = msg.thetadot
        if not self.initial_state_set:
            self.get_logger().info("Initial state is set.")
        self.initial_state_set = True

        self.p_0 = np.zeros(2)
        self.theta_0 = 0.
        self.theta_0 = msg.theta
        self.p_0[0] = msg.x
        self.p_0[1] = msg.y
        self.t_init_time = msg.timestamp
        self.t_init = self.t_init_time.sec + (self.t_init_time.nanosec/1e9)
        return

    def des_pose_callback(self, msg:Pose2D) -> None:
        self.p_d = np.zeros(2)
        self.theta_d = 0.
        self.p_d[0] = msg.x
        self.p_d[1] = msg.y
        self.theta_d = msg.theta
        self.desired_state_set = True
        self.get_logger().info("Desired pose is saved")
        return
    
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
                    self.trajectory.points.clear()
                    self.trajectory.joint_names.clear()
                    self.trajectory.joint_names.append("x")
                    self.trajectory.joint_names.append("y")

                    N = self.get_parameter("N").value
                    dt = self.get_parameter("dt").value
                    for k in range(N+1):
                        jtpt = JointTrajectoryPoint()
                        time_from_start = Duration()
                        time_from_start.sec = int(k*dt)
                        time_from_start.nanosec = int((k*dt - int(k*dt))*1.0e9)
                        jtpt.time_from_start = time_from_start
                        pxk = px[k]
                        pyk = py[k]
                        vxk = vx[k]
                        vyk = vy[k]
                        axk = ax[k]
                        ayk = ay[k]

                        jtpt.positions.append(pxk)
                        jtpt.positions.append(pyk)
                        jtpt.velocities.append(vxk)
                        jtpt.velocities.append(vyk)
                        jtpt.accelerations.append(axk)
                        jtpt.accelerations.append(ayk)
                        self.trajectory.points.append(jtpt)


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
            self.traj_publisher.publish(self.trajectory)
            self.get_logger().info("Trajectory is published")
            self.initial_state_set = False
            self.desired_state_set = False

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

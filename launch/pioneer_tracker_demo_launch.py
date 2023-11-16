import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController



def generate_launch_description():
    package_dir = get_package_share_directory('gmm_steer')

    robot_description = os.path.join(package_dir, 'resource', 'pioneer.urdf')

    webots_node = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'new_world.wbt'), 
        ros2_supervisor=True
    )

    # Ros2 supervisor
    ros2_supervisor = Ros2SupervisorLauncher()

    # Pioneer driver
    robot_driver = WebotsController(
        robot_name="My3AT",
        parameters=[
            {"robot_description":robot_description, 
             "use_sim_time":True}
        ]
    )

    # Robot state publisher
    state_publisher = Node(
        package="gmm_steer", 
        namespace="", 
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_name":"My3AT", 
             "use_sim_time":True}
        ]
    )

    # Trajectory Tracker
    # trajectory_tracker = Node(
    #     package="gmm_steer",
    #     namespace="", 
    #     executable="lyapunov_tracker", 
    #     name="lyapunov_tracker", 
    #     parameters=[
    #         {"robot_name":"My3AT", 
    #          "use_sim_time":True,
    #          "debug":True, 
    #          "k_x":0.02, 
    #          "k_y":0.5, 
    #          "k_theta":0.1}
    #     ]
    # )
    trajectory_tracker = Node(
        package="gmm_steer", 
        name="trajectory_tracker", 
        namespace="",
        executable="trajectory_tracker", 
        parameters=[
            {"robot_name":"My3AT", 
             "use_sim_time":True, 
             "debug":True}
        ]
    )

    # Trajectory Generator
    trajectory_generator = Node(
        package="gmm_steer", 
        namespace="", 
        executable="trajectory_generator", 
        name="trajectory_generator", 
        parameters=[
            {"robot_name":"My3AT", 
             "use_sim_time":True, 
             "max_speed":0.4,
             "dt":1.0, 
             "N":40}
        ]
    )

    bag_exec = ExecuteProcess(
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-a'],
        output="screen"
        )

    return LaunchDescription([
        webots_node,
        ros2_supervisor,
        robot_driver,
        state_publisher,
        trajectory_tracker,
        trajectory_generator, 
        bag_exec,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
import os
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

def generate_launch_description():

    package_name = "diffdrive"

    share_dir_path = get_package_share_directory(package_name)

    world_path = os.path.join(share_dir_path, 'worlds', 'gazebo_husky.world')

    gazebo_node = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
        output="screen"
        )

    return LaunchDescription([
        gazebo_node, 
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action = gazebo_node, 
            on_exit = [launch.actions.EmitEvent(event=launch.events.Shutdown())]
        ))
        ])
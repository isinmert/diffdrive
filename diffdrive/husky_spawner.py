import os
import sys
import rclpy
import math
import argparse

from ament_index_python import get_package_share_directory

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Quaternion

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()

    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q


def main():
    # Package name
    package_name = "diffdrive"
    # Get input arguments from the user
    parser = argparse.ArgumentParser()
    parser.add_argument("-name", type=str, default="Entity")
    parser.add_argument("-namespace", type=str, default="/demo")
    parser.add_argument("-x", type=float, default=0.)
    parser.add_argument("-y", type=float, default=0.0)
    parser.add_argument("-yaw", type=float, default=0.0)
    args = parser.parse_args()

    # Husky SDF file path
    husky_path = os.path.join(get_package_share_directory(package_name), 
                              'models', 'husky', 'model.sdf')
    
    # start ros2 client
    rclpy.init()

    # initialize node
    node = rclpy.create_node("husky_spawner")
    node.get_logger().info("Creating the client to connect /spawn_entity service")

    client = node.create_client(SpawnEntity, "/spawn_entity")

    # wait for service connection
    if not client.service_is_ready():
        node.get_logger().info("Waiting for service to be ready...")
    node.get_logger().info("Service is ready")

    # Get user defined values
    req_name = args.name
    robot_namespace = args.namespace
    x0 = args.x
    y0 = args.y
    yaw0 = args.yaw

    # Get initial quaternion 
    quat = quaternion_from_euler(0, 0, yaw0)

    # Set request 
    req = SpawnEntity.Request()
    req.name = req_name
    req.xml = open(husky_path, 'r').read()
    req.robot_namespace = robot_namespace
    req.initial_pose.position.x = x0
    req.initial_pose.position.y = y0
    req.initial_pose.orientation = quat

    node.get_logger().info("Sending spawn request")

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print("Future result : {}".format(future.result))
    else:
        raise RuntimeError(
            "Exception while calling the service : {}".format(future.exception)
            )
    
    node.get_logger().info("Done. Shutting down the node.")
    node.destroy_node()

    return

if __name__ == "__main__":
    main()
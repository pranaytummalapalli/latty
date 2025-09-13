from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import pathlib
import sys

import rclpy
from rclpy.node import Node as RclNode
from gazebo_msgs.srv import DeleteEntity

def delete_entity(name: str):
    rclpy.init()
    node = RclNode('delete_entity_launcher')
    client = node.create_client(DeleteEntity, '/delete_entity')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('/delete_entity service not available')
        rclpy.shutdown()
        return

    req = DeleteEntity.Request()
    req.name = name
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f'Successfully deleted entity [{name}]')
    else:
        node.get_logger().warn(f'Entity [{name}] may not exist')
    rclpy.shutdown()

def spawn_robot_if_model_exists(context, *args, **kwargs):
    
    env = kwargs.get('env', os.environ.copy())

    # Resolve package share directory
    pkg_share = get_package_share_directory('latty_chassis')
    model_file = os.path.join(pkg_share, 'models', 'latty_chassis_model', 'latty_model.sdf')
    
    print(f"[INFO] Using model file: {model_file}") 

    if not pathlib.Path(model_file).exists():
        print(f"[ERROR] Model file does not exist: {model_file}", file=sys.stderr)
        return []

    # Delete existing robot
    delete_entity('latty_chassis')

    # Spawn robot
    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'latty_chassis', '-file', model_file],
            output='screen',
            env=env
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('latty_chassis')
    world_path = os.path.join(pkg_share, 'world', 'lattys_world.sdf')

    # Set Gazebo model path
    gazebo_model_path = os.path.expanduser("~/.gazebo/gazebo_models")
    env = os.environ.copy()
    env['GAZEBO_MODEL_PATH'] = env.get('GAZEBO_MODEL_PATH', '') + ':' + gazebo_model_path  + ':' + os.path.join(pkg_share, 'models')

    # Launch Gazebo server (paused)
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', 
            '--verbose',
            '--paused',
            world_path, 
            '-s', 
            'libgazebo_ros_factory.so'
        ],
        output='screen',
        env=env
    )

    # Launch Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        env=env
    )

    # Spawn robot after 5 seconds if model exists
    spawn_robot = TimerAction(
        period=3.0,
        actions=[OpaqueFunction(function=spawn_robot_if_model_exists, kwargs={'env': env})]
    )

    return LaunchDescription([gzserver, gzclient, spawn_robot])

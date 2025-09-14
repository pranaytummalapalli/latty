from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

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

    pkg_share = get_package_share_directory('latty_chassis')
    urdf_file = os.path.join(pkg_share, 'urdf', 'latty_model.urdf.xacro')

    if not pathlib.Path(urdf_file).exists():
        print(f"[ERROR] URDF file does not exist: {urdf_file}", file=sys.stderr)
        return []

    delete_entity('latty_chassis')

    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'latty_chassis', '-topic', 'robot_description'],
            output='screen',
            env=env
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('latty_chassis')
    urdf_file = os.path.join(pkg_share, 'urdf', 'latty_model.urdf.xacro')
    world_path = os.path.join(pkg_share, 'world', 'lattys_world.sdf')
    controllers_file = os.path.join(pkg_share, 'config', 'latty_controllers.yaml')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    print("URDF path:", urdf_file)
    print("Controller File:", controllers_file)

    gazebo_model_path = os.path.expanduser("~/.gazebo/gazebo_models")
    env = os.environ.copy()
    env['GAZEBO_MODEL_PATH'] = (
        env.get('GAZEBO_MODEL_PATH', '')
        + ':' + gazebo_model_path
        + ':' + os.path.join(pkg_share, 'models')
    )

    # Publish robot state
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        env=env
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        env=env
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=spawn_robot_if_model_exists, kwargs={'env': env})]
    )

    # Start controller_manager after robot is spawned
    controller_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[ robot_description,
                    controllers_file ],
                output="screen"
            )
        ]
    )

    # Delay controller spawners
    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['front_wheel_steer_position_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        rsp_node,
        gzserver,
        gzclient,
        spawn_robot,
        # controller_manager,
        delayed_spawners
    ])

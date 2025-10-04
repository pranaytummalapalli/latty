from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
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
    rviz_config_file = os.path.join(pkg_share, "rviz", "latty.rviz")

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
        parameters=[robot_description, {"use_sim_time": True}],
        # remappings=[("/joint_states", "/joint_states_pos")]
    )

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so'
        ],
        output='screen',
        env=env
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        env=env
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'latty_chassis', '-topic', 'robot_description'],
        output='screen',
        env=env
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_front_wheel_steer_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_wheel_steer_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_right_wheel_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_left_wheel_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "latty.rviz")],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    latty_chassis_node = Node(
        package="latty_chassis",
        executable="move_latty",
        name="move_latty",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    latty_odom_node = Node(
        package="latty_chassis",
        executable="latty_odom",
        name="latty_odom",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
    rsp_node,
    gzserver,
    gzclient,
    spawn_robot,
    RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    ),
    RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_front_wheel_steer_position_controller],
        )
    ),
    RegisterEventHandler(
        OnProcessExit(
            target_action=load_front_wheel_steer_position_controller,
            on_exit=[load_right_wheel_velocity_controller],
        )
    ),
    RegisterEventHandler(
        OnProcessExit(
            target_action=load_right_wheel_velocity_controller,
            on_exit=[load_left_wheel_velocity_controller],
        )
    ),
    RegisterEventHandler(
        OnProcessExit(
            target_action=load_left_wheel_velocity_controller,
            on_exit=[latty_chassis_node, rviz_node, latty_odom_node],
        )
    ),
])
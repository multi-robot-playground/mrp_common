from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def prepare_launch(context):
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='')
    robot_name = LaunchConfiguration('robot_name')

    monitored_nodes = [
        "/robot0/controller_server",
        "/robot1/controller_server",
        "/robot2/controller_server",
        "/robot3/controller_server",
    ]
    heartbeat_interval = [
        float(1000),
        float(1000),
        float(1000),
        float(1000)
    ]

    load_nodes = GroupAction(
        actions=[
            Node(
                package='mrp_lifecycle_manager',
                executable='lifecycle_manager',
                output='screen',
                namespace=robot_name.perform(context),
                arguments=['--ros-args'],
                parameters=[{'node_names': monitored_nodes},
                            {'heartbeat_interval': heartbeat_interval}]
            )
        ]
    )

    return [
        robot_name_arg,
        load_nodes
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prepare_launch)
    ])

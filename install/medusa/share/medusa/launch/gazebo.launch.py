import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


import xacro

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('medusa'))
    xacro_file = os.path.join(pkg_path, 'urdf/medusa.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Node robot_state_publisher
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("gazebo_ros").find("gazebo_ros"),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # Spawn del robot más arriba
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '0.0',
            '-y', '-1.0',
            '-z', '0.5'   # <-- altura aumentada
        ],
        output='screen'
    )
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        load_joint_state_controller,
    ])

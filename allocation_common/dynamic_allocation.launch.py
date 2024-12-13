from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    ignition_gazebo_share = get_package_share_directory('ros_ign_gazebo')
    gazebo_description_share = get_package_share_directory('gazebo_description')
    control_terminal_share = get_package_share_directory('control_terminal')

    # Declare launch arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(gazebo_description_share, 'worlds', 'Allocation_Map.sdf'),
        description='Path to the Ignition Gazebo world file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Launch Ignition Gazebo with GUI'
    )

    # Ignition Gazebo launch
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ignition_gazebo_share, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={
            'ign_args': LaunchConfiguration('world_name'),
            'gui': LaunchConfiguration('gui'),
        }.items()
    )

    # Control terminal node
    control_terminal_node = Node(
        package='control_terminal',
        executable='control_terminal_node',
        name='control_terminal',
        output='screen',
        parameters=[{'robots_num': 0}]
    )

    # Launch description
    return LaunchDescription([
        world_name_arg,
        use_sim_time_arg,
        gui_arg,
        ignition_launch,
        control_terminal_node,
    ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Path to MoveIt configuration package
    moveit_config_dir = get_package_share_directory('manipulator_moveit_config')

    # Launch the MoveIt Planner
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py'))
    )

    # Launch the interface (RViz)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py'))
    )

    # Launch the geometry calculator (Robot State Publisher)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'rsp.launch.py'))
    )

    # PYTHON BRIDGE (It replaces ros2_control)
    bridge = Node(
        package='manipulator_core',
        executable='trajectory_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

# ADDED: Mixer for autonomous mode
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['/arm_joint_states']
        }]
    )

    return LaunchDescription([
        move_group,
        rviz,
        rsp,
        jsp_node,
        bridge
    ])
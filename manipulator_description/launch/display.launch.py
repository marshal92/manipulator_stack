import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('manipulator_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'display.rviz') # <-- Путь к конфигу

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] # <-- Загружаем конфиг
        )
    ])
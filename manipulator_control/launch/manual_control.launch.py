import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("manipulator_full", package_name="manipulator_moveit_config")
        .robot_description(file_path="config/manipulator_full.urdf.xacro")
        .robot_description_semantic(file_path="config/manipulator_full.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("manipulator_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # 1. Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        output="screen",
    )

    # 2. Rosbridge Websocket Node (for Web UI)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    # 3. Teleop Manager Node (Python backend with Mutex)
    teleop_manager_node = Node(
        package='manipulator_control',
        executable='teleop_manager',
        name='teleop_manager',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription(declared_arguments + [
        servo_node,
        rosbridge_node,
        teleop_manager_node
    ])

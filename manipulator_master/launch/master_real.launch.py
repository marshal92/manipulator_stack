import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # -------------------------------------------------------------
    # 1. Package directories
    # -------------------------------------------------------------
    manipulator_moveit_dir = get_package_share_directory('manipulator_moveit_config')

    # -------------------------------------------------------------
    # 2. Launch Arguments
    # -------------------------------------------------------------
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz')
    web_port_arg = DeclareLaunchArgument('port', default_value='8080', description='Web Server HTTP Port')

    # -------------------------------------------------------------
    # 3. Virtual Joint TF (world -> arm_base_link)
    # -------------------------------------------------------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_arm_base_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'arm_base_link'],
        parameters=[{'use_sim_time': False}]
    )

    # -------------------------------------------------------------
    # 4. Robot State Publisher (RSP)
    # -------------------------------------------------------------
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(manipulator_moveit_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # -------------------------------------------------------------
    # 5. Joint State Publisher (Mixer for /arm_joint_states -> /joint_states)
    # -------------------------------------------------------------
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['/arm_joint_states']
        }]
    )

    # -------------------------------------------------------------
    # 6. C++ Trajectory Bridge for Real Hardware (Action + Topic <-> ESP32)
    # -------------------------------------------------------------
    trajectory_bridge_node = Node(
        package='manipulator_control',
        executable='trajectory_bridge',
        name='trajectory_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # -------------------------------------------------------------
    # 7. MoveIt 2 (MoveGroup & RViz)
    # -------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("manipulator_full", package_name="manipulator_moveit_config")
        .robot_description(file_path="config/manipulator_full.urdf.xacro")
        .robot_description_semantic(file_path="config/manipulator_full.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(manipulator_moveit_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(manipulator_moveit_dir, 'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # -------------------------------------------------------------
    # 8. MoveIt Servo (Real-time Cartesian Jogging)
    # -------------------------------------------------------------
    servo_yaml_path = os.path.join(manipulator_moveit_dir, 'config', 'servo_config.yaml')
    with open(servo_yaml_path, 'r') as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = {'moveit_servo': servo_yaml}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': False}
        ],
        output='screen'
    )

    # -------------------------------------------------------------
    # 9. All 3 C++ Control Nodes from manipulator_control
    # -------------------------------------------------------------
    teleop_manager_node = Node(
        package='manipulator_control',
        executable='teleop_manager',
        name='teleop_manager',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    tactical_server_node = Node(
        package='manipulator_control',
        executable='tactical_server',
        name='tactical_server',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False}
        ]
    )

    control_hub_node = Node(
        package='manipulator_control',
        executable='control_hub',
        name='control_hub',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # -------------------------------------------------------------
    # 10. ROS Bridge & Web Server
    # -------------------------------------------------------------
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'max_message_size': 100000000
        }]
    )

    web_server_node = Node(
        package='manipulator_master',
        executable='web_server',
        name='web_server',
        output='screen',
        parameters=[{'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}]
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        rviz_arg,
        web_port_arg,
        static_tf_node,
        rsp_launch,
        jsp_node,
        trajectory_bridge_node,
        move_group,
        rviz,
        servo_node,
        teleop_manager_node,
        tactical_server_node,
        control_hub_node,
        rosbridge_node,
        web_server_node
    ])

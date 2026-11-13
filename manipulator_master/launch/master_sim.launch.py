import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    AppendEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # -------------------------------------------------------------
    # 1. Package directories
    # -------------------------------------------------------------
    manipulator_desc_dir = get_package_share_directory('manipulator_description')
    manipulator_moveit_dir = get_package_share_directory('manipulator_moveit_config')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # -------------------------------------------------------------
    # 2. Launch Arguments
    # -------------------------------------------------------------
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz')
    gz_gui_arg = DeclareLaunchArgument('gz_gui', default_value='true', description='Launch Gazebo GUI')
    web_port_arg = DeclareLaunchArgument('port', default_value='8080', description='Web Server HTTP Port')

    # -------------------------------------------------------------
    # 3. Environment & Gazebo Mesh Resource Path
    # -------------------------------------------------------------
    set_mesh_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(manipulator_desc_dir, '..')
    )

    # -------------------------------------------------------------
    # 4. Gazebo Simulator & Clock Bridge
    # -------------------------------------------------------------
    gz_args = PythonExpression([
        "'empty.sdf -r' if '", LaunchConfiguration('gz_gui'), "' == 'true' else '-s -r empty.sdf'"
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # -------------------------------------------------------------
    # 5. Robot State Publisher & Gazebo Spawner
    # -------------------------------------------------------------
    wrapper_path = os.path.join(manipulator_desc_dir, 'urdf', 'arm_sim_wrapper.urdf.xacro')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', wrapper_path]),
                value_type=str
            ),
            'use_sim_time': True
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'manipulator_arm', '-z', '0.0'],
        output='screen'
    )

    # Spawner for ros2_control controllers once entity spawned in Gazebo
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
                    output='screen'
                )
            ]
        )
    )

    # -------------------------------------------------------------
    # 6. MoveIt 2 (MoveGroup & RViz)
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
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(manipulator_moveit_dir, 'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # -------------------------------------------------------------
    # 7. MoveIt Servo (Real-time Cartesian Jogging)
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
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # -------------------------------------------------------------
    # 8. All 3 C++ Control Nodes from manipulator_control
    # -------------------------------------------------------------
    teleop_manager_node = Node(
        package='manipulator_control',
        executable='teleop_manager',
        name='teleop_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tactical_server_node = Node(
        package='manipulator_control',
        executable='tactical_server',
        name='tactical_server',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ]
    )

    control_hub_node = Node(
        package='manipulator_control',
        executable='control_hub',
        name='control_hub',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # -------------------------------------------------------------
    # 9. ROS Bridge & Web Server
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
        SetParameter(name='use_sim_time', value=True),
        rviz_arg,
        gz_gui_arg,
        web_port_arg,
        set_mesh_path,
        gazebo,
        clock_bridge,
        rsp_node,
        spawn_entity,
        load_controllers,
        move_group,
        rviz,
        servo_node,
        teleop_manager_node,
        tactical_server_node,
        control_hub_node,
        rosbridge_node,
        web_server_node
    ])

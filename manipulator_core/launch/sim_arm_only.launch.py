import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter   
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    moveit_config_dir = get_package_share_directory('manipulator_moveit_config')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    manipulator_desc_dir = get_package_share_directory('manipulator_description')

    set_mesh_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(manipulator_desc_dir, '..')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    wrapper_path = os.path.join(manipulator_desc_dir, 'urdf', 'arm_sim_wrapper.urdf.xacro')
    
    rsp = Node(
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

    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_arm_ctrl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        # =========================================================
        # Global Clock Synchronization for All Nodes in This Launch
        # =========================================================
        SetParameter(name='use_sim_time', value=True),
        
        set_mesh_path,
        gazebo,
        clock_bridge,
        rsp,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb, load_arm_ctrl]
            )
        ),
        move_group,
        rviz
    ])
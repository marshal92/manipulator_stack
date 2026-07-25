from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("manipulator").to_moveit_configs()
    
    tactical_server_node = Node(
        package="manipulator_control",
        executable="tactical_server",
        name="tactical_server", # Имя совпадает с node_name в MoveItPy
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            {'use_sim_time': True} 
        ],
    )

    return LaunchDescription([tactical_server_node])
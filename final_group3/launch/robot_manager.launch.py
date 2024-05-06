import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    
    moveit_package = get_package_share_directory('ariac_moveit_config')
    
    demo_cpp = Node(
        package="final_group3",
        executable="robot_manager",
        output="screen",
        parameters=generate_parameters(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_package, 'launch', 'ariac_robots_moveit.launch.py')
        )
    )
   

    
    
    timer_action_2 = TimerAction(
        period=0.0,
        actions=[moveit_launch]
    )
    
    timer_action_3 = TimerAction(
        period=3.0,
        actions=[demo_cpp]
    )
    
    nodes_to_start = [
        timer_action_2,
        timer_action_3,
    ]

    return nodes_to_start


def generate_launch_description():
    
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )

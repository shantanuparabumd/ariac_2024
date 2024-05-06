
#!/usr/bin/python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():


    yolo_sensor_interface = Node(
        package='final_group3',
        executable='advance_sensor_interface.py',
        output='screen'
    )
    
    sensor_manager = Node(
        package='final_group3',
        executable='sensor_manager_bonus',
        output='screen'
    )
    
    ccs_manager = Node(
        package='final_group3',
        executable='ccs_manager',
        output='screen'
    )

    # Add a timer to launch spawn_robot_world after 5 seconds
    timer_action_1 = TimerAction(
        period=0.0,
        actions=[yolo_sensor_interface]
    )
    
    timer_action_2 = TimerAction(
        period=16.0,
        actions=[sensor_manager]
    )
    
    timer_action_3 = TimerAction(
        period=15.0,
        actions=[ccs_manager]
    )

    # Launch Description 
    return LaunchDescription([
        timer_action_1,
        timer_action_2,
        timer_action_3
    ])


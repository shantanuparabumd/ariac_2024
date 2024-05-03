
#!/usr/bin/python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    sensor_manager = Node(
        package='rwa4_group3',
        executable='sensor_manager',
        output='screen'
    )
    
    ccs_manager = Node(
        package='rwa4_group3',
        executable='ccs_manager',
        output='screen'
    )


    timer_action_2 = TimerAction(
        period=1.0,
        actions=[sensor_manager]
    )
    
    timer_action_3 = TimerAction(
        period=2.0,
        actions=[ccs_manager]
    )

    # Launch Description 
    return LaunchDescription([
        timer_action_2,
        timer_action_3
    ])

    

    

    # Launch Description 
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_world
        
    ])

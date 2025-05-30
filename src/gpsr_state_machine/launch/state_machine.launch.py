from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpsr_state_machine',
            executable='sm_main.py',
            name='gpsr_state_machine',
            output='screen',
            parameters=[
                {
                    'speech_retry_limit': 3,
                    'max_retries': 3,
                }
            ]
        )
    ]) 
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2 topic pub --once /switchOn std_msgs/Bool "{data: true}"'],
            shell=True,
            output='screen'
        )
    ])
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Retrieve the bag file path from the launch configuration
    bag_file = LaunchConfiguration('bag_file')

    # Define the command to play the bag file
    play_bag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file],
        output='screen'
    )

    return LaunchDescription([
        play_bag_cmd
    ])

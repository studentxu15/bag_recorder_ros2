import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('bag_recorder')
    config_file = os.path.join(pkg_share, 'config', 'record_topics.yaml')

    home_dir = os.path.expanduser("~")
    bag_save_path = os.path.join(home_dir, '.ros/bag2')

    return LaunchDescription([
        Node(
            package='bag_recorder',
            executable='bag_recorder_node',
            name='bag_recorder_node',
            output='screen',
            parameters=[
                {'config_path': config_file},
                {'bag_save_path': bag_save_path},
                {'record_duration': -1},
                {'max_bag_size_mb': 100},
                {'topic_rate_limit': 0.0},
            ],
        ),
    ])
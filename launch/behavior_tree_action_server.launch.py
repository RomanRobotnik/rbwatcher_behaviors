from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory('rbwatcher_behaviors')
    default_tree = os.path.join(share_dir, 'config', 'default_tree.xml')

    return LaunchDescription([
        Node(
            package='rbwatcher_behaviors',
            executable='behavior_tree_action_server',
            name='rbwatcher_behavior_server',
            output='screen',
            parameters=[
                {
                    'default_tree_path': default_tree,
                    'bt_tick_frequency': 10.0,
                }
            ],
        )
    ])

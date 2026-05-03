import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('cognitive_amr_description')

    laser_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_multi_merger',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'laser_merger.yaml')
        ],
        remappings=[]
    )

    return LaunchDescription([
        laser_merger_node,
    ])

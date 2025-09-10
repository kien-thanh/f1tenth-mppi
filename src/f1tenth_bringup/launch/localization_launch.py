from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': get_package_share_directory('f1tenth_bringup') + '/maps/test_map.yaml'}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[get_package_share_directory('f1tenth_bringup') + '/config/amcl_params.yaml']
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
    ])
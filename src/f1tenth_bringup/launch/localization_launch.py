from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    map_yaml_file_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map YAML file to load'
    )

    map_yaml_file = LaunchConfiguration('map')

    return LaunchDescription([
        map_yaml_file_arg,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml_file}]
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
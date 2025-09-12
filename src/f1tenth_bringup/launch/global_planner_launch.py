from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():
    map_yaml_file_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map YAML file to load'
    )

    map_yaml_file = LaunchConfiguration('map')

    return LaunchDescription([
        map_yaml_file_arg,
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[get_package_share_directory('f1tenth_bringup') + '/config/planner_params.yml']
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['planner_server']}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_egoracecar_baselink',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'ego_racecar/base_link', 'base_link']
    )

    ])
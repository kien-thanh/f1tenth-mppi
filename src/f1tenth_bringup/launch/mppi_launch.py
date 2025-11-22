from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mppi",
                executable="mppi_node",
                name="mppi_node",
                parameters=[
                    get_package_share_directory("f1tenth_bringup")
                    + "/config/mppi_params.yml"
                ],
                output="screen",
            )
        ]
    )

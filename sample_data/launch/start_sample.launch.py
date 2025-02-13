from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the nodes to be launched
    sample_node = Node(
        package='sample_data',
        executable='sample',
        name='sample_node',
        output='screen'
    )

    # node2 = Node(
    #     package='your_package_name',
    #     executable='your_node_executable2',
    #     name='your_node_name2',
    #     output='screen'
    # )

    # Add the nodes to the LaunchDescription
    return LaunchDescription([
        sample_node
    ])